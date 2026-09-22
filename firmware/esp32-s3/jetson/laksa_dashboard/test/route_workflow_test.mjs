import assert from 'node:assert/strict';
import {followRouteUi, hasValidRoute, heroJourneyAvailable, maskContains} from '../web/route_workflow.mjs';

const bits = new Uint8Array([0b00110100]);
assert.equal(maskContains(bits, 4, 2, 2, 0), true);
assert.equal(maskContains(bits, 4, 2, 1, 0), false);
assert.equal(maskContains(bits, 4, 2, 4, 0), false);

const route = {state: 'PATH_READY', path: [[0.13, 0.27, 0], [0.91, 0.44, 0.2]], follow_available: true};
assert.equal(hasValidRoute(route), true);
assert.equal(heroJourneyAvailable(route), true);
assert.deepEqual(followRouteUi(route), {disabled: false, label: 'FOLLOW ROUTE'});
assert.deepEqual(followRouteUi({state: 'READY', path: []}), {disabled: true, label: 'FOLLOW ROUTE'});
assert.deepEqual(followRouteUi({...route, state: 'FOLLOWING'}), {disabled: false, label: 'CANCEL FOLLOW'});

console.log('route workflow tests passed');
