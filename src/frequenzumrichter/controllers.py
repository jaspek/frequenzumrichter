"""Controller building blocks: PI controller, limiter and filter.

The classes defined here form the discrete-time basic building blocks of the
control of a variable-frequency drive. All controllers operate with a fixed
sampling time ``dt``, which is passed on every call to :meth:`step`.
"""

from __future__ import annotations

from dataclasses import dataclass, field

__all__ = ["PIController", "RateLimiter", "LowPassFilter"]


@dataclass
class PIController:
    """Discrete-time PI controller with control-output saturation and anti-windup.

    The controller implements the control law

    .. math::

        u(t) = K_p \, e(t) + K_i \int e(\\tau)\, d\\tau

    Integration uses the forward Euler method. To counter integrator windup,
    *back-calculation* (anti-windup via feedback of the saturation difference)
    is used: as soon as the control output runs into the limit, the integral
    term is back-calculated accordingly.

    Parameters
    ----------
    kp:
        Proportional gain.
    ki:
        Integral gain [1/s].
    output_min, output_max:
        Lower/upper limit of the control output. ``None`` => unbounded.
    anti_windup_gain:
        Gain ``Kaw`` of the back-calculation. A common choice is ``1/Tt`` with
        the reset time ``Tt``. ``0`` disables anti-windup (pure clamping
        remains active, provided limits are set).
    """

    kp: float
    ki: float
    output_min: float | None = None
    output_max: float | None = None
    anti_windup_gain: float = 0.0
    integral: float = field(default=0.0)
    _last_output: float = field(default=0.0, repr=False)

    def reset(self, integral: float = 0.0) -> None:
        """Resets the integral store (and thus the controller)."""
        self.integral = integral
        self._last_output = 0.0

    def step(self, error: float, dt: float, feedforward: float = 0.0) -> float:
        """Computes one controller step.

        Parameters
        ----------
        error:
            Control deviation ``setpoint - actual value``.
        dt:
            Sampling time [s].
        feedforward:
            Optional feedforward term that is added before saturation
            (e.g. decoupling terms in field-oriented control).

        Returns
        -------
        float
            The limited control output.

        Notes
        -----
        :attr:`integral` holds the integral term already weighted by ``Ki``
        in control-output units. The control output is first formed from the
        current integral store and limited; afterwards the store is updated
        via back-calculation. If the control output runs into saturation, the
        term ``Kaw·(u_sat - u_unsat)`` slows further integration
        (anti-windup).
        """
        unbounded = self.kp * error + self.integral + feedforward
        bounded = self._clamp(unbounded)

        # Update integrator using classical back-calculation
        self.integral += (
            self.ki * error + self.anti_windup_gain * (bounded - unbounded)
        ) * dt

        self._last_output = bounded
        return bounded

    def _clamp(self, value: float) -> float:
        if self.output_min is not None and value < self.output_min:
            return self.output_min
        if self.output_max is not None and value > self.output_max:
            return self.output_max
        return value


@dataclass
class RateLimiter:
    """Limits the rate of change of a signal (ramp).

    Typically used to convert setpoint steps (speed, frequency) into
    smooth ramps and thus avoid current/torque surges.

    Parameters
    ----------
    rate_up, rate_down:
        Maximum rising/falling rate [unit/s]. When ``None``, ``rate_down``
        uses the same value as ``rate_up``.
    """

    rate_up: float
    rate_down: float | None = None
    value: float = 0.0

    def reset(self, value: float = 0.0) -> None:
        self.value = value

    def step(self, target: float, dt: float) -> float:
        rate_down = self.rate_up if self.rate_down is None else self.rate_down
        max_up = self.rate_up * dt
        max_down = rate_down * dt
        delta = target - self.value
        if delta > max_up:
            delta = max_up
        elif delta < -max_down:
            delta = -max_down
        self.value += delta
        return self.value


@dataclass
class LowPassFilter:
    """Discrete-time first-order PT1 low-pass filter.

    Realizes ``G(s) = 1 / (1 + s*tau)`` via forward Euler discretization.
    """

    tau: float
    value: float = 0.0

    def reset(self, value: float = 0.0) -> None:
        self.value = value

    def step(self, input_value: float, dt: float) -> float:
        if self.tau <= 0.0:
            self.value = input_value
            return self.value
        alpha = dt / (self.tau + dt)
        self.value += alpha * (input_value - self.value)
        return self.value
