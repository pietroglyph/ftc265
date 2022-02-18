# The Origins
I have been working with ftc265 for a while and encountered numerous errors and weird things when it came to using the `setPose` function. I believe I have found the problems, sources of the error, and the fixes for them.

## 1) `setPose` is weird.
The reason that sometimes `setPose` doesn't work if you do it too quickly after initialization is as follows:

The camera begins initialization at `new T265Camera()`.\
Then, `setPose` is ran, but initialization isn't complete, so `setPose` defaults to using `new Pose2d()` as the "origin"
for translation.\
Then, the camera fully initializes and whatever the position it started at is used, with the incorrect offset from
`setPose`.

### Example:
(Where there is something that looks like `(0,0)`, it means `(x,y)`. I ignore rotation for simplicity's sake, but it
acts the same)\
Initialization begins\
`setPose(0,0)` is ran\
Camera gives a reference of `(0,0)` because it has yet to initialize fully\
The offset is none because the origin and new pose for `setPose` were the same\
The camera fully initializes at a position of `(10,2)` and that is passed to the program because there is no offset.

### The Fix:
Wait until `getLastReceivedUpdate()` returns actual numbers and not just `(0,0)`, that way `setPose` has actual values
to calculate off.

## 2) `setPose` is broken.
I have no idea how no one has noticed this and asked about it or even explored it for themselves, as I did, but I have
found that the `setPose` function is truly broken.

The reason that `setPose()` does weird things when you try to use it multiple times is because it uses the translated
position in the offset calculation instead of the camera's position

That makes no sense so,
### Explanation:
There are 4 values in this example
```
Camera Pos = The position that the actual hardware camera believes it's at (not affected by offset)
Offset Pos = The position that is added to the camera pos to get the translated pos
Trans Pos = The position after (Camera Pos) + (Offset Pos), this is given to the user
New Pos = The position for calculating the offset pos (used as setPose(New Pos))
```
Also, something like `(x1,y1) - (x2,y2)` is the same as `(x1-x2, y1-y2)`. The same goes when using names instead of numbers.

The way the camera currently works is that `Offset Pos` is calculated by doing `(New Pos) - (Trans Pos)`.\
This poses problems.

Given:
```
Camera Pos = (10,2)
Offset Pos = (0,0)
(10,2) + (0,0):
Trans Pos = (10,2)
```

If we then set `New Pos` to be `(0,0)`, then the values change to be:
```
Camera Pos = (10,2)
Offset Pos = (-10,-2)
(10,2) + (-10,-2):
Trans Pos = (0,0)
```
This is normal!

However, if you then set `New Pos` to be `(0,0)` again, then the calculation is done **with the translated pos** causing
the offset to go back to `(0,0)` because `(New Pos) - (Trans Pos) = (0,0)`, which gives this:
```
Camera Pos = (10,2)
Offset Pos = (0,0)
(10,2) + (0,0):
Trans Pos = (10,2)
```

Therefore, `setPose` is broken in the library.

### The fix:
If the camera instead calculates `Offset Pos` by doing `(New Pos) - (Camera Pos)`, we get what we want, always making
`(Trans Pos) = (New Pos)` when `New Pos` is changed.

These example take place in a world where the camera stays perfectly still. If you introduce movement, the calculations
get even more broken.

Example for fun :P
```
Camera Pos = (6,7)
Offset Pos = (0,0)
(6,7) + (0,0):
Trans Pos = (6,7)
```
Run `setPose(0,0)` and you get:
```
Camera Pos = (6,7)
Offset Pos = (-6,-7)
(6,7) + (-6,-7):
Trans Pos = (0,0)
```

Then, move the camera by `(+2, -1)` and the values become:
```
Camera Pos = (8,6)
Offset Pos = (-6,-7)
(8,6) + (-6,-7)
Trans Pos = (2,-1)
```

Try doing `setPose(0,0)` again, and offset is calculated `(New Pos) - (Trans Pos) = (Offset Pos)`, or `(0,0) - (2,-1) = (-2,1)`, making the values after `setPose`:
```
Camera Pos = (8,6)
Offset Pos = (-2,1)
(8,6) + (-2,1):
Trans Pos = (6,7)
```
With `Trans Pos` not being `(0,0)` this concludes that `setPose()` is **fundamentally broken**.

### My edits:
I added a variable called `mLastRecievedCameraUpdate` (spelled incorrectly to match the incorrect spelling of `mLastRecievedCameraUpdate`) [here](https://github.com/Sparib/ftc265/blob/master/lib/src/main/java/com/spartronics4915/lib/T265Camera.java#L90).
I also added a consumer for that variable called `cameraConsumer` [here](https://github.com/Sparib/ftc265/blob/master/lib/src/main/java/com/spartronics4915/lib/T265Camera.java#L95-L100). \
I then created a new function called `consumeCameraUpdate` which takes the same parameters as `consumePoseUpdate` [here](https://github.com/Sparib/ftc265/blob/master/lib/src/main/java/com/spartronics4915/lib/T265Camera.java#L363-L374), and that is called by the C++ file ([here](https://github.com/Sparib/ftc265/blob/master/lib/src/main/cpp/t265wrapper.cpp#L226)). \
After being called it passes all of its arguments to `consumeCameraUpdate`, which I will probably soon change to it passing the pose instead of `x`, `y`, and `radians`.\
The final modification was making `setPose` use `mLastRecievedCameraUpdate` instead of `mLastRecievedUpdate` ([here](https://github.com/Sparib/ftc265/blob/master/lib/src/main/java/com/spartronics4915/lib/T265Camera.java#L339-L341)).
`mLastRecievedCameraUpdate` **is not** meant to be sent to the end user, thus the absence of any getter function.