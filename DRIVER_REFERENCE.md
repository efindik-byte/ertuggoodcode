# REBUILT 2026 — Driver & Operator Reference Sheet

---

## CONTROLLER MAP (Xbox Layout)

### DRIVER (Port 0) — Primary Control

```
              [LB] Feeder Reverse        [RB] Feeder (manual)
              [LT] Intake               [RT] Shooter (auto-speed)

  [D-pad Up]    Intake Arm UP       [Y] Auto-Aim + Auto-Drive to Hub
  [D-pad Down]  Intake Arm DOWN     [X] AprilTag Align (hub tags only)
  [D-pad Right] + RT = FULL POWER   [A] AUTO-CYCLE (toggle on/off)
                                    [B] CANCEL auto-cycle

  [Left Stick]  Drive (field-centric)    [Right Stick]  Rotate
  [Left Stick Press]  Reset heading      [Any Stick]    Override auto-cycle
```

### OPERATOR (Port 1) — Same Layout, Takes Priority When Active

The operator controller has the same button mapping. When the operator
touches any stick, trigger, or button, they take priority over the driver.
Control returns to the driver when the operator goes hands-off.

---

## THREE WAYS TO PLAY

### Mode 1: FULL AUTO (Recommended)
**Press A once.** The robot does EVERYTHING by itself:
1. Drives to the depot and picks up fuel
2. Drives to the hub at ideal shooting distance
3. Aims at the hub automatically
4. Spins shooter to the correct speed for the distance
5. Waits for hub to be ACTIVE
6. Feeds fuel when aimed + at speed + hub active
7. Goes back to depot and repeats

**To stop:** Press A again, press B, or touch any stick.

### Mode 2: SEMI-AUTO (Y + RT)
You drive manually, but scoring is automated:
1. **You drive** to the hub area using the sticks
2. **Hold Y** — robot auto-aims and auto-drives to ideal distance
3. **Hold RT** — shooter spins to the correct speed
4. **Feeder engages automatically** when all conditions are met:
   - Shooter at target RPM
   - Hub is ACTIVE
   - Robot is aimed within 5° of hub
   - Vision is working

### Mode 3: FULL MANUAL
Drive with sticks. Hold RT to shoot. Press RB to feed. You control everything.

---

## ALLIANCE-SPECIFIC INFO

### BLUE ALLIANCE
| Element         | Field Position (x, y meters) | Location Description         |
|-----------------|------------------------------|------------------------------|
| Blue Hub        | (4.03, 4.04)                 | Left-center of field         |
| Blue Depot      | (2.00, 6.50)                 | Near blue wall, left side    |
| Blue Scoring    | Computed at ideal distance from hub, facing hub |

### RED ALLIANCE
| Element         | Field Position (x, y meters) | Location Description         |
|-----------------|------------------------------|------------------------------|
| Red Hub         | (12.51, 4.04)                | Right-center of field        |
| Red Depot       | (14.50, 6.50)                | Near red wall, left side     |
| Red Scoring     | Computed at ideal distance from hub, facing hub |

**NOTE:** Depot positions are approximate defaults. Your programmer should
measure and update these for your specific strategy. Change them in code:
`kBlueDepotPosition` and `kRedDepotPosition` in RobotContainer.java.

---

## HUB SHIFT SCHEDULE (REBUILT 2026)

```
 Match Time    Phase              Hub Status
 ──────────    ─────              ──────────
 2:40 → 2:20  AUTONOMOUS         Both hubs ACTIVE
 2:20 → 2:10  Transition         Both hubs ACTIVE
 2:10 → 1:45  Shift 1 (25s)      One alliance active, one inactive *
 1:45 → 1:20  Shift 2 (25s)      Swaps
 1:20 → 0:55  Shift 3 (25s)      Swaps back
 0:55 → 0:30  Shift 4 (25s)      Swaps again
 0:30 → 0:00  ENDGAME            Both hubs ACTIVE
```

**\* Which alliance goes inactive first depends on who scored more in auto.**
- Win auto → your hub goes INACTIVE first (opponent scores first)
- Lose auto → your hub stays ACTIVE first (you score first)

**The controller RUMBLES 3 seconds before your hub goes active.** Use this
to pre-position near the hub so you score immediately when the shift changes.

---

## DASHBOARD WIDGETS — What to Watch

### Critical (put these front and center)

| Widget                   | What It Means                                        |
|--------------------------|------------------------------------------------------|
| **Hub Active**           | GREEN = score now! RED = collect fuel or play defense |
| **Shift Timer (s)**      | Seconds until the next shift change (-1 = no shifts) |
| **Fuel Scored (est)**    | Estimated fuel scored this match                     |
| **Fuel to Energized**    | How many more fuel to hit the 100-fuel RP            |
| **Fuel to Supercharged** | How many more fuel to hit the 360-fuel RP            |
| **Auto-Cycle Active**    | Is the robot running the automatic collect/score loop |

### Shooter Status

| Widget                | What It Means                                           |
|-----------------------|---------------------------------------------------------|
| **Shooter At Speed**  | TRUE = flywheel is at target RPM, safe to shoot         |
| **Shooter Target RPS**| What speed the shooter is trying to hold                |
| **Shooter Actual RPS**| What speed the shooter is actually spinning             |
| **Auto Shooter %**    | Calculated power level based on distance                |

### Targeting Status

| Widget                | What It Means                                           |
|-----------------------|---------------------------------------------------------|
| **Hub Distance (m)**  | How far you are from the hub                            |
| **Aim Error (deg)**   | How far off your heading is from the hub (0 = perfect)  |
| **Aimed**             | TRUE = heading is within 2° of the hub                  |
| **Vision Valid**       | TRUE = Limelight has seen AprilTags this match          |
| **Auto-Feed Active**  | TRUE = auto-feed conditions met, feeder is running      |

### Warnings

| Widget                      | What It Means                                    |
|-----------------------------|--------------------------------------------------|
| **WARNING: Hub Inactive!**  | You're trying to shoot but hub won't accept fuel |
| **RP Energized (100)**      | TRUE = you've hit the 100-fuel ranking point     |
| **RP Supercharged (360)**   | TRUE = you've hit the 360-fuel ranking point     |

---

## RANKING POINTS CHEAT SHEET

| RP Name        | Requirement              | Strategy                          |
|----------------|--------------------------|-----------------------------------|
| **Energized**  | Score 100 fuel (alliance)| Keep cycling — 100 is reachable   |
| **Supercharged**| Score 360 fuel (alliance)| All 3 robots must cycle fast      |
| **Traversal**  | 50 tower points (alliance)| We don't climb — allies need this|
| **Win**        | Most total points        | Score fuel + don't get penalized  |

---

## TUNING QUICK REFERENCE

All tuning values are on SmartDashboard under "Tune/..." and take effect
IMMEDIATELY — no redeploy needed.

| Dashboard Key            | What It Controls                | Default |
|--------------------------|---------------------------------|---------|
| Tune/Ideal Distance (m)  | Where the robot parks to shoot  | 2.5     |
| Tune/Shooter A           | Base shooter speed fraction     | 0.35    |
| Tune/Shooter B           | Distance scaling coefficient    | 0.18    |
| Tune/Shooter Max RPS     | Max flywheel speed (rot/sec)    | 80.0    |
| Tune/Aim Kp              | How aggressively it aims        | 3.0     |
| Tune/Range Kp            | How aggressively it drives to range | 1.5 |

**After tuning, write down the final values and give them to your programmer
so they can update the code defaults (survives reboot).**

---

## PRE-MATCH CHECKLIST

1. [ ] Power on robot, connect to WiFi
2. [ ] Open SmartDashboard or Shuffleboard
3. [ ] Verify **Vision Valid** shows TRUE
4. [ ] Verify **Hub Distance (m)** shows a reasonable number
5. [ ] Check alliance color is correct on Driver Station
6. [ ] Select autonomous mode (currently: Left_trench)
7. [ ] Verify both controllers are connected (Ports 0 and 1)
8. [ ] Reset heading if needed (left stick press)

---

## TROUBLESHOOTING

| Problem                         | Fix                                          |
|---------------------------------|----------------------------------------------|
| Vision Valid stays FALSE        | Check Limelight power, pipeline set to AprilTag, camera offsets configured |
| Robot won't auto-aim            | Need Vision Valid = TRUE first. Check Limelight |
| Shooter speed inconsistent      | Check Shooter Target vs Actual RPS — should match closely |
| Auto-cycle won't start          | PathPlanner must be configured. Check console for errors |
| Robot drives wrong direction    | Press left stick to reset field-centric heading |
| Hub Active always TRUE          | Normal in practice mode (no FMS game data)   |
| Controller not rumbling         | Only rumbles during real matches with FMS timing data |
| Auto-feed won't engage          | Need: Y held + RT held + at speed + hub active + aimed + vision valid |

---

*Generated for Team — REBUILT 2026 Season*
