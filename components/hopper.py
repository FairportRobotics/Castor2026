"""
components/hopper.py – HopperSubsystem ported to MagicBot component.

Hardware:
  - TalonFX  spindexer motor
  - TalonFX  kicker motor
"""

import magicbot
from phoenix6.configs import MotorOutputConfigs, Slot0Configs, TalonFXConfiguration
from phoenix6.hardware import TalonFX

import constants


class HopperComponent:
    """MagicBot component that controls the hopper (spindexer + kicker)."""

    def setup(self) -> None:
        """
        MagicBot lifecycle hook — instantiate and configure all hopper hardware.

        Called once by MagicBot after all component objects are created but
        before the robot enters any enabled mode.
        """
        # ── Spindexer ──────────────────────────────────────────────────────
        self._spindexer_motor = TalonFX(constants.HopperConstants.SPINDEXER_MOTOR_ID)
        spindexer_cfg = TalonFXConfiguration()
        spindexer_cfg.motor_output = MotorOutputConfigs().with_inverted(
            constants.HopperConstants.SPINDEXER_MOTOR_DIRECTION
        )
        spindexer_cfg.slot0 = Slot0Configs().with_k_p(0).with_k_i(0).with_k_d(0).with_k_s(0)
        self._spindexer_motor.configurator.apply(spindexer_cfg)

        # ── Kicker ─────────────────────────────────────────────────────────
        self._kicker_motor = TalonFX(constants.HopperConstants.KICKER_MOTOR_ID)
        kicker_cfg = TalonFXConfiguration()
        kicker_cfg.motor_output = MotorOutputConfigs().with_inverted(
            constants.HopperConstants.KICKER_MOTOR_DIRECTION
        )
        kicker_cfg.slot0 = Slot0Configs().with_k_p(0).with_k_i(0).with_k_d(0).with_k_s(0)
        self._kicker_motor.configurator.apply(kicker_cfg)

        self._kicker_velocity = self._kicker_motor.get_velocity()

    # ── Public API ─────────────────────────────────────────────────────────────

    def feed_kicker(self) -> None:
        """Drive the kicker motor forward at 75 % output to push a note into the launcher."""
        self._kicker_motor.set(0.75)

    def reverse_kicker(self) -> None:
        """Drive the kicker motor in reverse at 50 % output to un-jam a stuck note."""
        self._kicker_motor.set(-0.5)

    def stop_kicker(self) -> None:
        """Stop the kicker motor."""
        self._kicker_motor.stop_motor()

    def spindexer_on(self) -> None:
        """Drive the spindexer forward at 75 % output to advance notes toward the kicker."""
        self._spindexer_motor.set(0.75)

    def spindexer_reverse(self) -> None:
        """Drive the spindexer in reverse at 75 % output to clear a jam."""
        self._spindexer_motor.set(-0.75)

    def spindexer_off(self) -> None:
        """Stop the spindexer motor."""
        self._spindexer_motor.stop_motor()

    # ── Telemetry ──────────────────────────────────────────────────────────────

    @magicbot.feedback
    def kicker_speed_rps(self) -> float:
        return self._kicker_velocity.refresh().value_as_double

    # ── MagicBot lifecycle ─────────────────────────────────────────────────────

    def execute(self) -> None:
        # No periodic logic required; all control is driven by external callers.
        pass
