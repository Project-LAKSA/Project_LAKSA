export function maskContains(bits, width, height, column, row) {
  if (!bits || column < 0 || row < 0 || column >= width || row >= height) return false;
  const index = row * width + column;
  return Boolean(bits[index >> 3] & (1 << (index & 7)));
}

export function hasValidRoute(planning) {
  return Array.isArray(planning?.path) && planning.path.length > 1 &&
    ['PATH_READY', 'FOLLOWING', 'FOLLOW_FAILED'].includes(planning?.state);
}

export function heroJourneyAvailable(planning) {
  return hasValidRoute(planning) && planning.state === 'PATH_READY';
}

export function followRouteUi(planning) {
  if (planning?.state === 'FOLLOWING') return {disabled: false, label: 'CANCEL FOLLOW'};
  if (planning?.state === 'STARTING_ROUTE') return {disabled: true, label: 'STARTING ROUTE…'};
  if (planning?.state === 'PLANNING') return {disabled: true, label: 'PLANNING…'};
  const ready = hasValidRoute(planning) && planning?.follow_available === true;
  return {disabled: !ready, label: 'FOLLOW ROUTE'};
}
