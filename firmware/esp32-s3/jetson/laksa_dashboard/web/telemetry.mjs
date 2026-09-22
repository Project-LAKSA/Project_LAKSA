export class LatestFrameMailbox {
  constructor(types = ['pose', 'cloud', 'lidar_slice', 'safe_goal_mask', 'occupancy']) {
    this.allowed = new Set(types);
    this.latest = new Map();
    this.coalesced = Object.fromEntries(types.map(type => [type, 0]));
  }

  push(type, payload) {
    if (!this.allowed.has(type)) throw new Error(`Unsupported ephemeral type: ${type}`);
    if (this.latest.has(type)) this.coalesced[type] += 1;
    this.latest.set(type, payload);
  }

  take(type) {
    const value = this.latest.get(type);
    this.latest.delete(type);
    return value;
  }

  clear() { this.latest.clear(); }

  get pending() { return this.latest.size; }
}

export class ReconnectGate {
  constructor(setTimer = (...args) => setTimeout(...args), clearTimer = id => clearTimeout(id)) {
    this.generation = 0;
    this.timer = null;
    this.setTimer = setTimer;
    this.clearTimer = clearTimer;
  }

  begin() {
    this.generation += 1;
    if (this.timer !== null) this.clearTimer(this.timer);
    this.timer = null;
    return this.generation;
  }

  owns(generation) { return generation === this.generation; }

  schedule(generation, callback, delayMs) {
    if (!this.owns(generation) || this.timer !== null) return false;
    this.timer = this.setTimer(() => {
      if (!this.owns(generation)) return;
      this.timer = null;
      callback();
    }, delayMs);
    return true;
  }
}
