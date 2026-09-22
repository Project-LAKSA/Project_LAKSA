import assert from 'node:assert/strict';
import {boundedBackingSize, fitDistanceForBox, mapToWorld, worldToMap} from '../web/geometry.mjs';

function grid(x, y, yaw, resolution = 0.05) {
  return {origin: [x, y, 0, 0, 0, Math.sin(yaw / 2), Math.cos(yaw / 2)], resolution, width: 200, height: 100};
}

for (const g of [grid(0, 0, 0), grid(2.3, -1.7, 0), grid(-4, 3, 0.73)]) {
  for (const point of [[0, 0], [12.4, 9.2], [199.5, 99.5]]) {
    const world = mapToWorld(g, ...point), roundTrip = worldToMap(g, world.x, world.y);
    assert.ok(Math.abs(roundTrip.x - point[0]) < 1e-9);
    assert.ok(Math.abs(roundTrip.y - point[1]) < 1e-9);
    for (const dpr of [1, 2, 3]) {
      const scale = 1.7, panX = 31, panY = -17;
      const canvasX = panX + roundTrip.x * scale, canvasY = panY + (g.height - roundTrip.y) * scale;
      const back = mapToWorld(g, (canvasX - panX) / scale, g.height - (canvasY - panY) / scale);
      assert.ok(Math.hypot(back.x - world.x, back.y - world.y) < 1e-9);
      assert.ok(boundedBackingSize(390, 700, dpr, 4_000_000, 4096).width <= 4096);
    }
  }
}

for (const [width, height] of [[1, 1], [10, 10], [20, 5], [5, 20]]) {
  for (const aspect of [390 / 700, 700 / 390, 1440 / 900]) {
    const fov = 55 * Math.PI / 180, distance = fitDistanceForBox(width, height, 0, fov, aspect, 1.12);
    assert.ok(Number.isFinite(distance) && distance > 0);
    const radius = Math.hypot(width, height) / 2;
    const vertical = Math.atan2(radius, distance), horizontal = Math.atan(Math.tan(fov / 2) * aspect);
    assert.ok(vertical <= Math.min(fov / 2, horizontal) / 1.01);
  }
}

console.log('dashboard geometry tests: PASS');
