"""PID controller with the practical bits a noisy GPS loop needs."""
from __future__ import annotations


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class PID:
    """PID with integral clamp, filtered derivative and derivative-on-measurement.

    * The derivative acts on the *measurement*, not the error, so a step in the
      setpoint (e.g. the avoidance team changing the lateral offset) does not
      produce a steering kick.
    * The derivative is low-pass filtered (time constant ``d_tau`` seconds)
      because differentiating 5-10 Hz GPS positions is very noisy.
    * The integral contribution is clamped to +/- ``i_limit`` and can be frozen
      by the caller while the actuator is saturated (anti-windup).
    """

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        i_limit: float = 0.0,
        out_limit: float = 1.0,
        d_tau: float = 0.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = abs(i_limit)
        self.out_limit = abs(out_limit)
        self.d_tau = max(0.0, d_tau)
        self.reset()

    def reset(self) -> None:
        self._prev_meas = None
        self._d_filt = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.output = 0.0

    def update(
        self,
        error: float,
        dt: float,
        measurement: float | None = None,
        freeze_integral: bool = False,
    ) -> float:
        """Advance one step. ``measurement`` defaults to ``-error``."""
        if measurement is None:
            measurement = -error

        self.p_term = self.kp * error

        if self.ki != 0.0 and not freeze_integral:
            self.i_term = clamp(
                self.i_term + self.ki * error * dt, -self.i_limit, self.i_limit
            )
        elif self.ki == 0.0:
            self.i_term = 0.0

        if self._prev_meas is None or dt <= 0.0:
            raw_rate = 0.0
        else:
            raw_rate = (measurement - self._prev_meas) / dt
        self._prev_meas = measurement
        alpha = dt / (self.d_tau + dt) if (self.d_tau + dt) > 0.0 else 1.0
        self._d_filt += alpha * (raw_rate - self._d_filt)
        self.d_term = -self.kd * self._d_filt

        self.output = clamp(
            self.p_term + self.i_term + self.d_term, -self.out_limit, self.out_limit
        )
        return self.output
