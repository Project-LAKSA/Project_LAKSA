import assert from 'node:assert/strict';
import {LatestFrameMailbox, ReconnectGate} from '../web/telemetry.mjs';

const mailbox = new LatestFrameMailbox();
for (let sequence = 1; sequence <= 5; sequence += 1) mailbox.push('pose', {sequence});
assert.equal(mailbox.take('pose').sequence, 5);
assert.equal(mailbox.coalesced.pose, 4);
assert.equal(mailbox.pending, 0);

// A heavy cloud pending at the same time cannot replace or erase the newest pose.
mailbox.push('cloud', {sequence: 1, points: new Array(45000).fill(0)});
mailbox.push('pose', {sequence: 6});
mailbox.push('pose', {sequence: 7});
assert.equal(mailbox.take('pose').sequence, 7);
assert.equal(mailbox.take('cloud').sequence, 1);

let nextTimer = 1;
const timers = new Map();
const gate = new ReconnectGate((callback) => {
  const id = nextTimer; nextTimer += 1; timers.set(id, callback); return id;
}, id => timers.delete(id));
const first = gate.begin();
assert.equal(gate.schedule(first, () => {}, 1500), true);
assert.equal(gate.schedule(first, () => {}, 1500), false);
const second = gate.begin();
assert.equal(timers.size, 0);
assert.equal(gate.owns(first), false);
assert.equal(gate.owns(second), true);
assert.equal(gate.schedule(first, () => {}, 1500), false);
assert.equal(gate.schedule(second, () => {}, 1500), true);
assert.equal(timers.size, 1);

console.log('dashboard telemetry-flow tests: PASS');
