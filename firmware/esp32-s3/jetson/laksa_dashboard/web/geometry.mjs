export function originYaw(grid) {
  const q = grid.origin;
  return Math.atan2(2 * (q[6] * q[5] + q[3] * q[4]), 1 - 2 * (q[4] * q[4] + q[5] * q[5]));
}

export function mapToWorld(grid, mapX, mapY) {
  const angle = originYaw(grid), x = mapX * grid.resolution, y = mapY * grid.resolution;
  return {x: grid.origin[0] + Math.cos(angle) * x - Math.sin(angle) * y,
    y: grid.origin[1] + Math.sin(angle) * x + Math.cos(angle) * y};
}

export function worldToMap(grid, worldX, worldY) {
  const angle = originYaw(grid), dx = worldX - grid.origin[0], dy = worldY - grid.origin[1];
  return {x: (Math.cos(angle) * dx + Math.sin(angle) * dy) / grid.resolution,
    y: (-Math.sin(angle) * dx + Math.cos(angle) * dy) / grid.resolution};
}

export function boundedBackingSize(cssWidth, cssHeight, requestedDpr, maxPixels, maxDimension) {
  const width = Math.max(1, cssWidth), height = Math.max(1, cssHeight);
  const pixelDpr = Math.sqrt(maxPixels / (width * height));
  const dimensionDpr = Math.min(maxDimension / width, maxDimension / height);
  const dpr = Math.max(0.25, Math.min(requestedDpr, pixelDpr, dimensionDpr));
  return {width: Math.max(1, Math.round(width * dpr)), height: Math.max(1, Math.round(height * dpr)), dpr};
}

export function fitDistanceForBox(sizeX, sizeY, sizeZ, verticalFovRadians, aspect, padding = 1.12) {
  const radius = Math.max(0.001, 0.5 * Math.hypot(sizeX, sizeY, sizeZ));
  const verticalHalfFov = Math.max(0.01, verticalFovRadians / 2);
  const horizontalHalfFov = Math.atan(Math.tan(verticalHalfFov) * Math.max(0.01, aspect));
  const limitingHalfFov = Math.max(0.01, Math.min(verticalHalfFov, horizontalHalfFov));
  return radius * padding / Math.sin(limitingHalfFov);
}
