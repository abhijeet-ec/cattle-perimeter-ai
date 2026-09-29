# Cattle Perimeter AI --- Motion Detection Implementation

## 1. Why Motion Detection Was Added

FOMO provides an object bounding box:

``` text
x, y, width, height
```

A single bounding box tells us **where the animal is**, but not directly
whether it is moving.

The project therefore uses the bounding-box position across consecutive
inference results to estimate movement.

This is **software-based motion tracking**. It does not use a PIR
sensor, accelerometer or separate motion sensor.

## 2. Basic Idea

For every accepted cattle detection:

``` text
Bounding box
     ↓
Calculate center
     ↓
Compare with previous center
     ↓
Calculate ΔX and ΔY
     ↓
Ignore tiny changes
     ↓
Choose dominant direction
     ↓
Require consecutive confirmation
     ↓
Output direction
```

The center is calculated as:

``` text
centerX = x + width / 2
centerY = y + height / 2
```

The next detection is compared against the previous center.

## 3. Movement Parameters

The firmware uses:

``` cpp
#define MOVEMENT_THRESHOLD 8.0f
#define REQUIRED_MOVEMENT_FRAMES 2
```

### Movement threshold

A change smaller than 8 pixels is treated as detection jitter rather
than real movement.

``` cpp
if (
    absX < MOVEMENT_THRESHOLD &&
    absY < MOVEMENT_THRESHOLD
) {
    // STATIONARY
}
```

This prevents small changes in the FOMO bounding box from constantly
producing movement events.

### Required movement frames

A direction must be confirmed over **2 consecutive frames** before it
becomes the reported direction.

This reduces one-frame direction changes caused by noisy detections.

## 4. Direction Calculation

The firmware calculates:

``` cpp
deltaX = currentCenterX - previousCenterX;
deltaY = currentCenterY - previousCenterY;
```

Then it compares:

``` text
|deltaX| vs |deltaY|
```

The larger component determines the dominant direction.

### Horizontal

``` text
deltaX > 0 → RIGHT
deltaX < 0 → LEFT
```

### Vertical

``` text
deltaY > 0 → DOWN
deltaY < 0 → UP
```

If both changes are below the movement threshold:

``` text
STATIONARY
```

## 5. Direction Codes

The firmware also exposes a numeric direction code:

    Code Direction
  ------ ------------
       0 STATIONARY
       1 LEFT
       2 RIGHT
       3 UP
       4 DOWN

The shared variable is:

``` cpp
volatile int movementDirectionCode = 0;
```

This allows another source file, such as `app_httpd.cpp`, to access the
current movement state.

## 6. First Detection

The first valid detection is not immediately treated as movement.

It establishes the reference point:

``` cpp
previousCenterX = currentCenterX;
previousCenterY = currentCenterY;
entryZone = currentZone;
movementDirection = MOVEMENT_STATIONARY;
movementDirectionCode = 0;
```

This gives the system an **entry position**.

## 7. Zone Detection

The tracking frame is 160 pixels wide.

The horizontal position is divided into three zones:

``` text
| LEFT | CENTER | RIGHT |
```

The logic is approximately:

``` cpp
if (centerX < TRACKING_FRAME_WIDTH / 3.0f)
    LEFT;

else if (centerX > TRACKING_FRAME_WIDTH * 2.0f / 3.0f)
    RIGHT;

else
    CENTER;
```

The system therefore stores:

-   `currentZone`
-   `entryZone`

Example:

``` text
Entry Zone: CENTER
Current Zone: RIGHT
```

This gives the dashboard more information than a simple LEFT/RIGHT
movement label.

## 8. Consecutive-Frame Confirmation

The algorithm keeps a counter:

``` cpp
uint8_t stableMovementFrames = 0;
```

If the candidate direction stays the same:

``` cpp
stableMovementFrames++;
```

If the candidate changes:

``` cpp
stableMovementFrames = 1;
```

Only after:

``` cpp
stableMovementFrames >= REQUIRED_MOVEMENT_FRAMES
```

does the direction become the confirmed movement direction.

With the current configuration:

``` text
2 matching frames → confirmed movement
```

## 9. Reset Behaviour

When the bounding-box movement is below the threshold:

``` text
STATIONARY
```

and the movement frame counter is reset.

When tracking itself is reset, the firmware clears:

``` text
previous center
stable frame count
direction
direction code
current zone
entry zone
```

and returns them to their initial state.

## 10. Verified Result

The current firmware produced a real detection:

``` text
cattle: 0.93
x:40 y:88 w:32 h:32

Center: X=56.0 Y=104.0
Movement: STATIONARY
Current Zone: CENTER
Entry Zone: CENTER
DIRECTION: STATIONARY
```

The detection also passed the strict 90% confidence threshold:

``` text
Confidence: 92.97%
```

So the complete chain was verified:

``` text
FOMO detection
     ↓
Bounding box
     ↓
Center calculation
     ↓
Zone calculation
     ↓
Movement tracking
     ↓
Direction output
```

## 11. Important Limitation

This is **object-position tracking**, not optical-flow motion detection.

The system decides that an animal moved when the detected bounding-box
center changes between inference results.

Therefore, camera movement, unstable detections or major bounding-box
changes can affect the movement estimate.

For the current ESP32-CAM project, this approach was selected because it
requires no additional motion sensor and uses information already
produced by FOMO.
