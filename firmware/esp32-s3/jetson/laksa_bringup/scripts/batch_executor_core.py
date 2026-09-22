"""Fail-closed, ROS-free state machine for a finite authorized batch."""
from __future__ import annotations
from batch_characterization_manifest import validate

class BatchAbort(RuntimeError): pass

class Lease:
    def __init__(self, max_age_s, now): self.max_age_s=max_age_s; self.last=now
    def renew(self, now): self.last=now
    def valid(self, now): return now-self.last <= self.max_age_s

class Executor:
    """Calls injected I/O only after manifest, lease, and precheck approval."""
    def __init__(self, manifest, phrase, lease, now, precheck, run, neutral, analyze):
        validate(manifest,phrase); self.manifest=manifest; self.lease=lease; self.now=now
        self.precheck,self.run,self.neutral,self.analyze=precheck,run,neutral,analyze
        self.completed=[]; self.stopped=None
    def abort(self, reason):
        self.neutral(); self.stopped=reason; raise BatchAbort(reason)
    def execute(self):
        for item in self.manifest['trials']:
            if not self.lease.valid(self.now()): self.abort('SESSION_LEASE_EXPIRED_BEFORE_TRIAL')
            ok,reason=self.precheck(item)
            if not ok: self.abort(reason)
            try:
                self.run(item,lambda: self.lease.valid(self.now()))
            except BatchAbort: self.abort('SESSION_LEASE_EXPIRED_DURING_TRIAL')
            self.neutral()
            if not self.lease.valid(self.now()): self.abort('SESSION_LEASE_EXPIRED_AFTER_NEUTRAL')
            if not self.analyze(item): self.abort('ANALYSIS_OR_INGEST_FAILED')
            self.completed.append((item['stage'],item['variant']))
        return self.completed
