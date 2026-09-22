#!/usr/bin/env python3
"""One interactive, fail-closed ±900 characterization campaign on Jetson."""
from __future__ import annotations
import argparse, datetime as dt, json, signal, subprocess, sys, time
from pathlib import Path

import yaml

ROOT=Path(__file__).resolve().parent
AUTO_VARIANTS={'T21A':'left','T22':'forward','T23':'level_1','T24':'level_1','T25':'positive','T26':'left_low','T27':'forward_low'}
ROS_COMMAND_TIMEOUT_S=10
GATE_PROBE_TIMEOUT_S=30

def strategic_repeats():
    cfg=yaml.safe_load((ROOT/'../config/supervised_characterization_protocols.yaml').resolve().read_text()) or {}
    return cfg.get('strategic_repeats',{}).get('T31',[])

def stamp(): return dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
def run(argv, *, check=True, timeout=None):
    try:
        return subprocess.run(argv, check=check, timeout=timeout)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(f'command timed out after {timeout}s: {error.cmd}') from error
def say(level,text): print(f'[{level}] {text}',flush=True)

def ros(*args, check=True):
    try:
        return subprocess.run(['ros2',*args],text=True,capture_output=True,check=check,timeout=ROS_COMMAND_TIMEOUT_S)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(f'ROS graph query timed out after {ROS_COMMAND_TIMEOUT_S}s: {error.cmd}') from error

def set_enabled(value):
    try:
        result=subprocess.run(['ros2','topic','pub','--once','/laksa/characterization_enable','std_msgs/msg/Bool',f'{{data: {str(bool(value)).lower()}}}'],text=True,capture_output=True,timeout=ROS_COMMAND_TIMEOUT_S)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(f'characterization gate update timed out after {ROS_COMMAND_TIMEOUT_S}s') from error
    if result.returncode: raise RuntimeError('could not set characterization safety gate')

def verify_gate_lifecycle():
    result=run([sys.executable,str(ROOT/'characterization_gate_probe.py')],check=False,timeout=GATE_PROBE_TIMEOUT_S)
    if result.returncode: raise RuntimeError('characterization gate lifecycle probe failed')

def preflight():
    nodes=[]
    for _ in range(20):
        nodes=ros('node','list',check=False).stdout.splitlines()
        if '/drive_supervisor' in nodes: break
        time.sleep(.5)
    if '/drive_supervisor' not in nodes: raise RuntimeError('drive_supervisor is not present after startup wait')
    enable_info=ros('topic','info','/laksa/characterization_enable',check=False).stdout
    if 'std_msgs/msg/Bool' not in enable_info: raise RuntimeError('installed supervisor lacks characterization safety gate')
    status_info=ros('topic','info','/laksa/characterization_enabled',check=False).stdout
    if 'std_msgs/msg/Bool' not in status_info: raise RuntimeError('supervisor does not expose characterization gate state')
    info=ros('topic','info','/laksa/command','-v').stdout
    if 'drive_supervisor' not in info: raise RuntimeError('/laksa/command is not owned by drive_supervisor')
    req=ros('topic','info','/laksa/characterization_request').stdout
    if 'laksa_interfaces/msg/DriveCommand' not in req: raise RuntimeError('characterization request topic/type is unavailable')
    say('OK','ROS supervisor graph verified')

def quality(session: Path, direction: str):
    analysis=session/'analysis.json'
    if not analysis.exists(): return {'status':'DATA_QUALITY_FAIL','reasons':['analysis_missing']}
    report=json.loads(analysis.read_text()); trial=(report.get('trials') or [{}])[0]
    raw=[]
    for p in (session/'trials').glob('*/raw_bag/*.jsonl'):
        raw += [json.loads(x) for x in p.read_text().splitlines() if x.strip()]
    vesc=[r.get('data',{}) for r in raw if r.get('topic')=='/laksa/vesc/state']
    target=float(trial.get('target_erpm') or 0); sign=1 if target>0 else -1
    requested=[float(x.get('requested_erpm',0)) for x in vesc]; active=[float(x.get('active_erpm',0)) for x in vesc]; measured=[float(x.get('measured_erpm',0)) for x in vesc]
    reasons=[]
    if trial.get('metadata',{}).get('status')!='COMPLETE': reasons.append('trial_not_complete')
    if len(vesc)<12: reasons.append('insufficient_vesc_samples')
    if not requested or max((sign*x for x in requested),default=0)<.8*abs(target): reasons.append('requested_target_not_observed')
    if not active or max((sign*x for x in active),default=0)<.5*abs(target): reasons.append('active_command_not_observed')
    if not measured or max((sign*x for x in measured),default=0)<.1*abs(target): reasons.append('measured_response_not_observed')
    if any(int(x.get('fault_code',0))!=0 for x in vesc): reasons.append('vesc_fault')
    if any(not bool(x.get('telemetry_fresh',False)) for x in vesc): reasons.append('telemetry_stale')
    response=trial.get('response',{})
    if response.get('status')!='OBSERVABLE': reasons.append('response_unobservable')
    if response.get('time_to_near_zero_s') is None: reasons.append('neutral_return_unobservable')
    return {'status':'DATA_QUALITY_PASS' if not reasons else 'DATA_QUALITY_FAIL','direction':direction,'target_erpm':target,'vesc_samples':len(vesc),'peak_requested_erpm':sign*max((sign*x for x in requested),default=0),'peak_active_erpm':sign*max((sign*x for x in active),default=0),'peak_measured_erpm':sign*max((sign*x for x in measured),default=0),'reasons':reasons,'analysis':str(analysis)}

def prompt(text, allowed):
    while True:
        value=input(text).strip().upper()
        if value in allowed:return value
        say('INFO',f'Enter one of: {", ".join(sorted(allowed))}')

def trial(campaign: Path, direction: str):
    say('INFO',f'T10 {direction.upper()} ±900: deterministic, independently armed.')
    before=set(campaign.glob('T10_AUTOMATED_V1_*'))
    result=run([sys.executable,str(ROOT/'automated_characterization_runner.py'),'--direction',direction,'--output-root',str(campaign)],check=False)
    after=set(campaign.glob('T10_AUTOMATED_V1_*')); created=sorted(after-before)
    if not created:
        raise RuntimeError(f'{direction} runner did not initialize a trial directory')
    metadata=yaml.safe_load((created[-1]/'trials'/f'trial_01_{direction}_900'/'metadata.yaml').read_text()) if (created[-1]/'trials'/f'trial_01_{direction}_900'/'metadata.yaml').exists() else {}
    if metadata.get('status') in ('ABORTED','QUIT'):
        reason=metadata.get('abort_reason') or 'UNSPECIFIED_ABORT'
        report={'status':metadata['status'],'direction':direction,'abort_reason':reason,'trial_dir':str(created[-1]),'analysis_available':bool(metadata.get('analysis_available',False))}
        path=campaign/'reports'/f't10_{direction}.json';path.write_text(json.dumps(report,indent=2)+'\n')
        say('FAIL' if metadata['status']=='ABORTED' else 'INFO',f'T10 {direction} {metadata["status"].lower()}: {reason}')
        return report
    if result.returncode:
        raise RuntimeError(f'{direction} runner exited {result.returncode} without structured abort metadata')
    report=quality(created[-1],direction); path=campaign/'reports'/f't10_{direction}.json';path.write_text(json.dumps(report,indent=2)+'\n')
    say('OK' if report['status']=='DATA_QUALITY_PASS' else 'FAIL',f"{report['status']}: {', '.join(report['reasons']) or 'usable evidence'}")
    return report

def t20_complete(campaign: Path):
    """T20 is immutable evidence; only its explicit COMPLETE marker advances T21."""
    for path in sorted(campaign.glob('T20_SUPERVISED_*/T20_static_steering.yaml')):
        try:
            data=yaml.safe_load(path.read_text()) or {}
            if data.get('status') == 'COMPLETE': return path
        except Exception:
            pass
    return None

def planned_stage(campaign: Path):
    """Select the next evidence gap; completed trials are never replayed."""
    from twin_characterization_roadmap import next_test
    row=next_test(campaign)
    return row['test_id'] if row else 'NONE'


def _interrupt_to_cleanup(_signum, _frame):
    """Turn terminal/SSH loss into the existing fail-safe finally path."""
    raise KeyboardInterrupt

def main(argv=None):
    p=argparse.ArgumentParser(description='LAKSA one-command supervised characterization campaign')
    p.add_argument('--output-root',type=Path,default=Path('/home/ubuntu/laksa_vehicle_id'));p.add_argument('--dry-run',action='store_true');p.add_argument('--skip-t20',action='store_true');p.add_argument('--resume-campaign',type=Path);a=p.parse_args(argv)
    # The launcher runs this through an SSH PTY.  SIGINT, SIGTERM and SIGHUP
    # must execute ``finally`` so the supervisor's characterization gate is
    # explicitly disabled rather than relying only on request expiry.
    original_handlers = {
        signum: signal.getsignal(signum)
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    }
    for signum in original_handlers:
        signal.signal(signum, _interrupt_to_cleanup)
    resume=a.resume_campaign
    if resume is None:
        for candidate in sorted(a.output_root.glob('campaign_*'),reverse=True):
            reports=candidate/'reports'
            try:
                f=json.loads((reports/'t10_forward.json').read_text());r=json.loads((reports/'t10_reverse.json').read_text());m=yaml.safe_load((candidate/'campaign_metadata.yaml').read_text()) or {}
                if f.get('status')=='DATA_QUALITY_PASS' and r.get('status')=='DATA_QUALITY_PASS' and t20_complete(candidate):
                    resume=candidate;break
            except Exception: pass
    if resume:
        campaign=resume;meta=yaml.safe_load((campaign/'campaign_metadata.yaml').read_text()) or {};stage=planned_stage(campaign);meta['status']=f'RESUMING_{stage}';meta['resumed_utc']=dt.datetime.now(dt.timezone.utc).isoformat();say('OK',f'RESUME_{stage} {campaign}; preserving valid completed evidence')
    else:
        campaign=a.output_root/f'campaign_{stamp()}';(campaign/'reports').mkdir(parents=True);meta={'schema_version':'laksa-one-command-campaign-v1','campaign_id':campaign.name,'started_utc':dt.datetime.now(dt.timezone.utc).isoformat(),'tests':['T10_FORWARD_900','T10_REVERSE_900'],'status':'BOOTSTRAP'}
    (campaign/'campaign_metadata.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
    try:
        if a.dry_run:
            if resume:
                if stage=='T21_IMU_SANITY': run([sys.executable,str(ROOT/'t21_imu_sanity_runner.py'),'--dry-run'])
                elif stage in AUTO_VARIANTS:
                    run([sys.executable,str(ROOT/'supervised_stage_runner.py'),'--test',stage,'--variant',AUTO_VARIANTS[stage],'--dry-run'])
                elif stage=='T31':
                    for item in strategic_repeats(): run([sys.executable,str(ROOT/'supervised_stage_runner.py'),'--test',item['test'],'--variant',item['variant'],'--dry-run'])
                elif stage in ('T21B','T28','T29','T30_SENSOR_MODEL','T31','FINAL_TWIN_VALIDATION'):
                    run([sys.executable,str(ROOT/'characterization_stage_runner.py'),'--test',stage,'--dry-run'])
                run([sys.executable,str(ROOT/'twin_characterization_roadmap.py'),'--campaign',str(campaign),'--dry-run'])
                say('OK',f'DRY_RUN: campaign resume point is {stage}; no completed evidence rerun')
                return 0
            verify_gate_lifecycle()
            run([sys.executable,str(ROOT/'automated_characterization_runner.py'),'--direction','forward','--mock-arm'])
            say('OK','DRY_RUN: Bool-only gate lifecycle verified; no DriveCommand or motion requested');return 0
        # A powered vehicle and complete live ROS graph are required only for
        # physical execution, never for an offline/dry-run software check.
        preflight()
        if resume:
            if stage=='T21_IMU_SANITY':
                result=run([sys.executable,str(ROOT/'t21_imu_sanity_runner.py'),'--output-root',str(campaign)],check=False)
                meta['status']='COMPLETE_T21_IMU_SANITY' if result.returncode==0 else 'STOPPED_T21_IMU_SANITY';return 0 if result.returncode==0 else 2
            if stage in AUTO_VARIANTS:
                # One human ARM authorizes one bounded deterministic trial.
                # The runner only requests through drive_supervisor.
                result=run([sys.executable,str(ROOT/'supervised_stage_runner.py'),'--test',stage,'--variant',AUTO_VARIANTS[stage],'--output-root',str(campaign)],check=False)
            elif stage=='T31':
                # Strategic parent repeats are scheduled deterministically;
                # every child process still requires its own human ARM.
                result=None
                for item in strategic_repeats():
                    result=run([sys.executable,str(ROOT/'supervised_stage_runner.py'),'--test',item['test'],'--variant',item['variant'],'--output-root',str(campaign)],check=False)
                    if result.returncode: break
            else:
                result=run([sys.executable,str(ROOT/'characterization_stage_runner.py'),'--test',stage,'--output-root',str(campaign)],check=False)
            meta['status']=f'COMPLETE_{stage}' if result and result.returncode==0 else f'STOPPED_{stage}';return 0 if result and result.returncode==0 else 2
        forward=trial(campaign,'forward')
        if forward['status'] in ('ABORTED','QUIT'):
            meta['status']='QUIT' if forward['status']=='QUIT' else 'STOPPED_ABORT';meta['abort_reason']=forward.get('abort_reason');return 0 if forward['status']=='QUIT' else 2
        if forward['status']!='DATA_QUALITY_PASS': meta['status']='STOPPED_DATA_QUALITY';return 2
        say('INFO','LAKSA must be fully stopped. Reposition if needed.')
        if prompt('Type CONTINUE to prepare reverse, or QUIT: ',{'CONTINUE','QUIT'})!='CONTINUE': meta['status']='QUIT_AFTER_FORWARD';return 0
        reverse=trial(campaign,'reverse')
        if reverse['status']!='DATA_QUALITY_PASS': meta['status']='STOPPED_DATA_QUALITY';return 2
        comparison={'status':'PAIR_RECORDED_NOT_ASYMMETRY_PROVEN','forward':forward,'reverse':reverse,'note':'One pair cannot establish physical asymmetry; inspect timing-resolution bounds and repeat campaign.'}
        (campaign/'reports'/'forward_reverse_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
        if not a.skip_t20 and prompt('NEXT AVAILABLE TEST: T20 zero-traction steering. Type CONTINUE or QUIT: ',{'CONTINUE','QUIT'})=='CONTINUE':
            result=run([sys.executable,str(ROOT/'automated_steering_characterization_runner.py'),'--output-root',str(campaign)],check=False)
            if result.returncode: meta['status']='STOPPED_T20';return 2
        say('INFO','T21 IMU sanity is next; this is human rotation only and creates no traction/steering request.')
        result=run([sys.executable,str(ROOT/'t21_imu_sanity_runner.py'),'--output-root',str(campaign)],check=False)
        meta['status']='COMPLETE_T21_IMU_SANITY' if result.returncode==0 else 'STOPPED_T21_IMU_SANITY';return 0 if result.returncode==0 else 2
    except KeyboardInterrupt:
        meta['status']='ABORTED_OPERATOR_INTERRUPT';say('FAIL','Campaign interrupted; request will expire and parameter is being disabled');return 130
    except Exception as error:
        meta['status']='ABORTED';meta['failure']=str(error);say('FAIL',str(error));return 2
    finally:
        if not a.dry_run:
            try:set_enabled(False);say('OK','Characterization authority disabled')
            except Exception:say('FAIL','Could not disable characterization gate; supervisor heartbeat still expires requests')
        else: say('OK','DRY_RUN: no characterization gate publication')
        meta['finished_utc']=dt.datetime.now(dt.timezone.utc).isoformat();(campaign/'reports'/'campaign_summary.json').write_text(json.dumps(meta,indent=2)+'\n');(campaign/'campaign_metadata.yaml').write_text(yaml.safe_dump(meta,sort_keys=False));say('INFO',f'Campaign artifacts: {campaign}')
        for signum, handler in original_handlers.items():
            signal.signal(signum, handler)
if __name__=='__main__':raise SystemExit(main())
