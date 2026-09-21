# formations/

Shapes **drawn by hand** in the Console and saved here.

A built-in formation (`line`, `wedge`, `circle`, …) is a *function* of
(count, spacing), which is what lets one name serve a fleet of any size. A
saved formation is a *drawing*: a specific arrangement of a specific number of
vehicles, made by dragging them about the map until the arrangement was
satisfactory. Both are legitimate, and they are stored differently for exactly
that reason — a function has parameters, a drawing has coordinates.

What stays parametric is the **size**. Each file records the spacing it was
drawn at (the closest gap between any two of its vehicles), so requesting the
shape at another spacing scales every offset by the ratio. The shape is
preserved; only its size remains a decision made later.

Offsets are metres from the formation's **centre**, so placing a formation is
placing its middle. The ground station is never in one: every link in a run is
measured against where the bench is, and moving it because the fleet changed
shape would move the ruler along with the thing being measured.

Nothing is written here automatically. Setup → **Save formation as...** is the
only action that creates a file, so this folder holds exactly the shapes that
were judged worth keeping.
