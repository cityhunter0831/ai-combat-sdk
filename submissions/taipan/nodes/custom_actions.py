"""
Taipan Custom Action Nodes — v2.0
Callsign: Taipan

═══════════════════════════════════════════════════════════════
CHANGELOG (v1 → v2)
═══════════════════════════════════════════════════════════════

FIXED: Breaking-change bug — all angle reads no longer multiply by 180.0.
  Blackboard now stores ata_deg / tau_deg as REAL DEGREES (0–180 / -180–180).
  Old code: obs.get("ata_deg", 0.0) * 180.0  ← WRONG (range 0–32400)
  New code: obs.get("ata_deg", 0.0)           ← CORRECT (range 0–180)

ADDED: SmartEvasionAction — direction-aware + altitude-aware break turn.
  Replaces predictable built-in BreakTurn for ThreatResponse branch.

ADDED: GeometryBreakerAction — vertical displacement burst to shatter
  two-circle deadlock (Eagle1 counter). Uses TimedAction base class for
  clean RUNNING semantics and phase-based execution.

REMOVED: PNAttack — BANNED in competition (PNAttack is not GunAttack).
  Code is preserved as a comment block for reference only.

RETAINED: PNPursuit — fixed angle conversions; available for future BT
  iterations if needed (not currently referenced in taipan_v2.yaml).

═══════════════════════════════════════════════════════════════
SDK UNIT CONVENTIONS (BLACKBOARD_REFERENCE.md)
═══════════════════════════════════════════════════════════════
  ata_deg          : 0–180°     (0=nose-on, 180=rear)
  aa_deg           : 0–180°     (0=safe at enemy tail, 180=enemy nose-on)
  tau_deg          : -180–180°  (positive=enemy right, negative=enemy left)
  relative_bearing : -180–180°  (positive=enemy right, negative=enemy left)
  side_flag        : -1/0/1     (-1=enemy left, 0=front, 1=enemy right)
  alt_gap_ft       : signed ft  (positive=enemy ABOVE us, negative=we are above)
  distance_ft      : 0–65617 ft
  closure_rate_kts : signed kts (positive=closing, negative=opening)
  ego_altitude_ft  : 0–49213 ft
  ego_vc_kts       : 0–778 kts

WEZ PARAMETERS (wez_params.yaml):
  max_angle_deg    : 12.0°      (ATA must be < 12° for any damage)
  min_range_ft     : 500 ft
  max_range_ft     : 3000 ft
  base_dps         : 25 HP/s    (with angle + distance multipliers)

HARD DECK: 1000 ft (immediate loss on violation)
"""

import logging
import py_trees

# TimedAction base class provides RUNNING semantics with automatic
# step counting. duration_steps is written in 5 Hz reference units;
# the base class auto-scales to the real 10 Hz BT tick rate.
# Result: duration_steps=15 always equals 3 seconds of real time.
#
# The SDK ships as a compiled .pyd — Pylance cannot inspect it directly.
# The try/except provides a pure-Python stub for static analysis only;
# at runtime the real compiled class is always loaded from the except branch.
try:
    from src.behavior_tree.nodes.actions import TimedAction  # type: ignore[import]
except ImportError:
    class TimedAction(py_trees.behaviour.Behaviour):  # type: ignore[no-redef]
        """Pylance stub — real class loaded from compiled SDK at runtime."""

        def __init__(self, name: str = "TimedAction", duration_steps: int = 15) -> None:
            super().__init__(name)
            self.blackboard = self.attach_blackboard_client()
            self.blackboard.register_key(
                key="observation", access=py_trees.common.Access.READ
            )
            self.blackboard.register_key(
                key="action", access=py_trees.common.Access.WRITE
            )

        def set_action(
            self,
            delta_altitude_idx: int,
            delta_heading_idx: int,
            delta_velocity_idx: int,
        ) -> None: ...

        def on_start(self) -> None: ...

        def execute(self, step: int, total: int) -> None: ...

        def on_finish(self, status: py_trees.common.Status) -> None: ...

logger = logging.getLogger(__name__)


# ============================================================
# BaseAction — blackboard wiring for all custom action nodes
# ============================================================

class BaseAction(py_trees.behaviour.Behaviour):
    """Custom action base class — blackboard READ observation / WRITE action."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key="observation", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="action", access=py_trees.common.Access.WRITE
        )

    def set_action(
        self,
        delta_altitude_idx: int,
        delta_heading_idx: int,
        delta_velocity_idx: int,
    ):
        """Write [altitude, heading, velocity] discrete action to blackboard.

        Indices:
          delta_altitude  : 0=hard-descend  1=descend  2=hold  3=climb  4=hard-climb
          delta_heading   : 0=hard-left(-90°) ... 4=straight ... 8=hard-right(+90°)
          delta_velocity  : 0=hard-decel    1=decel    2=hold  3=accel  4=hard-accel
        """
        self.blackboard.action = [
            delta_altitude_idx,
            delta_heading_idx,
            delta_velocity_idx,
        ]


# ============================================================
# SmartEvasionAction
# ============================================================

class SmartEvasionAction(BaseAction):
    """Direction-aware and altitude-aware break turn for ThreatResponse.

    TACTICAL PROBLEM WITH BUILT-IN BreakTurn:
      BreakTurn always: hard-opposite-turn + DESCEND + accelerate.
      After 2–3 evasions, the opponent can predict the descent and
      pre-position below us. The pattern becomes exploitable.

    THIS CLASS ADDS:
      1. Altitude-awareness: if the enemy is ABOVE us (alt_gap_ft > 0),
         descending puts us closer to their WEZ geometry — instead we hold
         altitude or climb to deny the diving attack angle.
         If the enemy is BELOW (alt_gap_ft < 0), we are already above;
         climbing wastes the altitude advantage — we hold or descend to
         maintain the offensive geometry even while evading heading.

      2. Urgency scaling: uses closure_rate_kts to determine how
         aggressively to accelerate during the break.

    BREAK DIRECTION LOGIC:
      side_flag:  1 → enemy right → hard LEFT break (heading idx 0)
      side_flag: -1 → enemy left  → hard RIGHT break (heading idx 8)
      side_flag:  0 → dead ahead  → use tau_deg sign to pick side:
                                     tau ≥ 0 → enemy slightly right → LEFT
                                     tau < 0 → enemy slightly left  → RIGHT

    RETURNS: SUCCESS every tick (reactive — re-evaluated at 10 Hz).
    The ThreatResponse Sequence keeps calling this as long as InEnemyWEZ
    condition is true, providing continuous evasion while under threat.
    """

    # Altitude thresholds for vertical component selection
    _ALT_GAP_HIGH_FT = 300.0    # enemy is meaningfully ABOVE us
    _ALT_GAP_LOW_FT  = -300.0   # enemy is meaningfully BELOW us (we are above)

    # Closure rate threshold: above this, we need max acceleration to escape
    _HIGH_CLOSURE_KTS = 100.0

    def update(self) -> py_trees.common.Status:
        try:
            obs = self.blackboard.observation

            side_flag       = obs.get("side_flag",        0)
            tau_deg         = obs.get("tau_deg",          0.0)
            alt_gap_ft      = obs.get("alt_gap_ft",       0.0)
            closure_rate    = obs.get("closure_rate_kts", 0.0)

            # ── HEADING: break AWAY from enemy ──────────────────────
            if side_flag > 0:          # enemy on our right → break LEFT
                heading_idx = 0        # hard left (-90°)
            elif side_flag < 0:        # enemy on our left → break RIGHT
                heading_idx = 8        # hard right (+90°)
            else:                      # enemy dead ahead → use tau_deg
                heading_idx = 0 if tau_deg >= 0.0 else 8

            # ── ALTITUDE: 3D evasion based on relative altitude ─────
            if alt_gap_ft > self._ALT_GAP_HIGH_FT:
                # Enemy is above us → they may try a diving attack.
                # Do NOT descend (that closes vertical range).
                # Climb slightly to force them to steepen their dive angle.
                delta_alt = 3          # climb

            elif alt_gap_ft < self._ALT_GAP_LOW_FT:
                # We are above the enemy → we have altitude advantage.
                # Maintain or descend slightly — don't surrender height by climbing.
                delta_alt = 2          # hold altitude

            else:
                # Co-altitude: pure horizontal break turn is optimal.
                # Descending during a flat turn maximises turn rate
                # (gravity assist on the bank) — same as built-in BreakTurn.
                delta_alt = 1          # descend

            # ── VELOCITY: afterburner if high closure, else accel ───
            if closure_rate > self._HIGH_CLOSURE_KTS:
                delta_vel = 4          # hard-accel: gain separation fast
            else:
                delta_vel = 3          # normal accel during the turn

            self.set_action(delta_alt, heading_idx, delta_vel)
            return py_trees.common.Status.SUCCESS

        except Exception as exc:
            logger.warning(f"SmartEvasionAction error: {exc}")
            self.set_action(2, 4, 2)   # safe fallback: hold all
            return py_trees.common.Status.FAILURE


# ============================================================
# GeometryBreakerAction
# ============================================================

class GeometryBreakerAction(TimedAction):
    """Vertical displacement burst to break two-circle turning deadlock.

    TACTICAL PROBLEM — TWO-CIRCLE DEADLOCK:
      When both aircraft turn AWAY from each other (HCA > 90°, IsTwoCircle),
      each nose-to-tail cycle produces large ATA values (> 60°) on every
      pass. Neither aircraft can reduce ATA enough for a WEZ entry because
      both are already committed to the same horizontal turn radius.
      Result: an infinite flat circle — no damage is dealt by either side.

    SOLUTION — VERTICAL DISPLACEMENT (Boom & Zoom setup):
      Break the 2D geometry by climbing steeply out of the turning plane.
      The enemy, still in the horizontal circle, cannot follow immediately
      (they are already in a banked turn and committed to that energy state).
      After the climb establishes vertical separation:
        - We have altitude ADVANTAGE (potential energy stored)
        - The enemy is now BELOW us in horizontal plane
        - We can convert altitude → speed in a diving attack (VerticalApproach
          branch [8] then GunEngagement [2] / SnapShot [3] during the pass)

    EXECUTION PHASES (duration_steps=30 @ 5Hz ref = 6 seconds total):
      Phase 1 — CLIMB (steps 1 → 20, first 4 seconds):
        Hard climb (alt=4) + mild turn toward enemy (heading 5 or 3)
        + maintain speed (vel=3).
        Mild inward turn keeps the enemy roughly in front so we don't
        completely lose sight; the hard climb is the priority.

      Phase 2 — DIVE SETUP (steps 21 → 30, final 2 seconds):
        Begin descending (alt=1) + stronger turn toward enemy (heading 6 or 2)
        + full afterburner (vel=4).
        Converts stored altitude energy to closing speed for the dive pass.
        At the end of Phase 2, VerticalApproach [8] or GunEngagement [2]
        should take over as the distance/ATA conditions are met.

    YAML INTEGRATION:
      The parent Sequence in taipan_v2.yaml gates this action with:
        DistanceBelow(6000) + DistanceAbove(2500) + IsTwoCircle
      The IsTwoCircle condition replaces the old internal ATA > 60° guard.
      IsTwoCircle (HCA > 90°) is the canonical SDK indicator for two-circle
      geometry — more precise than our hand-crafted ATA threshold was.

    MEMORY SEMANTICS:
      Returns RUNNING during the burst. The parent Sequence uses memory=false
      (default), so conditions are re-evaluated each tick. If IsTwoCircle
      becomes false mid-burst (geometry improved!), the Sequence fails,
      on_finish(INVALID) is called to reset state, and higher-priority
      branches (GunEngagement etc.) take over immediately.

    TIMING REFERENCE:
      duration_steps=30 is in 5 Hz reference units.
      Actual BT tick rate is 10 Hz → 60 real ticks = 6 seconds.
      This gives enough vertical separation to exit the deadlock zone.
    """

    def __init__(
        self,
        name: str = "GeometryBreakerAction",
        duration_steps: int = 30,           # 5Hz ref = 6 seconds real time
    ):
        super().__init__(name=name, duration_steps=duration_steps)
        self._side: int = 0                 # captured at burst start

    def on_start(self) -> None:
        """Called once when the burst begins. Capture enemy side."""
        obs = self.blackboard.observation
        self._side = obs.get("side_flag", 0)
        logger.debug(
            f"GeometryBreakerAction: burst start | side_flag={self._side}"
        )

    def execute(self, step: int, total: int) -> None:
        """Called every BT tick during the burst. Must call set_action().

        Args:
            step  : current step, 1-indexed (1 … total)
            total : total steps for this burst (= duration_steps × scale)
        """
        # Phase boundary at 2/3 of total duration
        phase_1_end = (total * 2) // 3

        if step <= phase_1_end:
            # ── Phase 1: HARD VERTICAL CLIMB ─────────────────────
            # Mild turn toward enemy: keeps them in our forward hemisphere
            # but doesn't commit to a turning fight (that's the deadlock).
            # Heading 5 = slight right (+22.5°), heading 3 = slight left (-22.5°)
            mild_toward = 5 if self._side >= 0 else 3
            self.set_action(4, mild_toward, 3)   # hard-climb, mild-turn, accel

        else:
            # ── Phase 2: DIVE SETUP / ATTACK TRANSITION ──────────
            # Begin descending and turn more aggressively toward enemy.
            # Full afterburner converts stored altitude to closure rate.
            # Heading 6 = medium right (+45°), heading 2 = medium left (-45°)
            strong_toward = 6 if self._side >= 0 else 2
            self.set_action(1, strong_toward, 4) # descend, strong-turn, hard-accel

    def on_finish(self, status: py_trees.common.Status) -> None:
        """Called when burst completes or is externally interrupted."""
        logger.debug(
            f"GeometryBreakerAction: burst finished | status={status} | "
            f"side={self._side}"
        )
        self._side = 0   # reset for next activation


# ============================================================
# PNPursuit — Proportional Navigation enhanced pursuit
# FIXED: Removed * 180.0 conversion (Breaking Change from BLACKBOARD_REFERENCE)
# NOT currently referenced in taipan_v2.yaml; retained for future use.
# ============================================================

class PNPursuit(BaseAction):
    """PN-enhanced pursuit with energy management.

    Heading  : PD controller on tau_deg (real degrees, no conversion needed).
    Altitude : Situation-dependent (hard deck → climb, aligned → dive, etc.)
    Speed    : ATA-aware — max speed when pointing at enemy, decel in turns.

    BREAKING CHANGE NOTE:
      tau_deg and ata_deg are stored as REAL DEGREES in the blackboard.
      This class previously multiplied by 180.0 — that bug is now fixed.
    """

    def __init__(
        self,
        name: str = "PNPursuit",
        kp: float = 0.8,
        kd: float = 0.3,
        close_range_ft: float = 4921.0,   # 1500 m → ft
        wez_max_ft: float = 3000.0,        # real WEZ max (wez_params.yaml)
        wez_min_ft: float = 500.0,         # real WEZ min
        far_range_ft: float = 13123.0,     # 4000 m → ft
    ):
        super().__init__(name)
        self.kp = kp
        self.kd = kd
        self.close_range_ft = close_range_ft
        self.wez_max_ft = wez_max_ft
        self.wez_min_ft = wez_min_ft
        self.far_range_ft = far_range_ft
        self._prev_tau: float | None = None

    @staticmethod
    def _heading_pd(
        tau_deg: float, tau_rate: float, kp: float, kd: float
    ) -> int:
        """PD controller on tau → discrete heading index [0–8]."""
        cmd = kp * tau_deg + kd * tau_rate
        idx = int(round(cmd / 22.5)) + 4
        return max(0, min(8, idx))

    def update(self) -> py_trees.common.Status:
        try:
            obs = self.blackboard.observation

            # ── Values are already in degrees — NO * 180.0 ──────
            tau_deg      = obs.get("tau_deg",          0.0)
            ata_deg      = obs.get("ata_deg",          90.0)
            distance_ft  = obs.get("distance_ft",   32808.0)
            alt_gap_ft   = obs.get("alt_gap_ft",       0.0)
            altitude_ft  = obs.get("ego_altitude_ft", 15000.0)
            closure_kts  = obs.get("closure_rate_kts", 0.0)

            # ── HEADING: PD on tau ───────────────────────────────
            if self._prev_tau is not None:
                tau_rate = (tau_deg - self._prev_tau) / 0.1  # 10 Hz BT tick
                kp = self.kp * (1.5 if distance_ft < self.close_range_ft else 1.0)
                heading_idx = self._heading_pd(tau_deg, tau_rate, kp, self.kd)
            else:
                # First tick — proportional only
                cmd = self.kp * tau_deg
                heading_idx = max(0, min(8, int(round(cmd / 22.5)) + 4))
            self._prev_tau = tau_deg

            # ── ALTITUDE: situation-dependent ───────────────────
            if altitude_ft < 1300.0:            # approaching hard deck
                delta_alt = 4
            elif ata_deg < 30.0 and distance_ft > self.wez_max_ft:
                delta_alt = 1 if altitude_ft > 6562.0 else 2  # dive or hold
            elif ata_deg > 90.0:
                delta_alt = 2                   # enemy behind — hold altitude
            elif alt_gap_ft > 656.0:
                delta_alt = 2                   # enemy high — hold, don't expose
            elif alt_gap_ft > 328.0:
                delta_alt = 2
            else:
                delta_alt = 3                   # below enemy — climb for advantage

            # ── SPEED: ATA-aware ─────────────────────────────────
            if distance_ft < self.wez_min_ft:
                delta_vel = 0                   # too close — brake hard
            elif distance_ft < self.wez_max_ft and ata_deg < 12.0:
                delta_vel = 1                   # in WEZ — decel for stable shot
            elif ata_deg < 30.0 and distance_ft > self.close_range_ft:
                delta_vel = 4                   # aligned + far — sprint
            elif ata_deg > 60.0 and distance_ft < self.close_range_ft:
                delta_vel = 1                   # lost angle, close — slow down
            elif distance_ft < self.close_range_ft:
                delta_vel = 2                   # close but not aligned — hold
            elif distance_ft < self.far_range_ft:
                delta_vel = 3
            else:
                delta_vel = 4

            # Override: if opening rapidly and out of WEZ, sprint back
            if closure_kts < -50.0 and distance_ft > self.wez_max_ft:
                delta_vel = max(delta_vel, 4)

            self.set_action(delta_alt, heading_idx, delta_vel)
            return py_trees.common.Status.SUCCESS

        except Exception as exc:
            logger.warning(f"PNPursuit error: {exc}")
            self.set_action(2, 4, 2)
            return py_trees.common.Status.FAILURE


# ============================================================
# BANNED: PNAttack
# ============================================================
# PNAttack is NOT GunAttack. The competition rules ban PNAttack.
# Only GunAttack (built-in SDK node) is permitted for weapon use.
# Do NOT reference PNAttack in any YAML submitted to the tournament.
#
# class PNAttack(BaseAction):  ← BANNED — DO NOT USE
#     pass