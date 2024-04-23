# Automatic Autonomous Duplicator

As of early 2024, PathPlanner has no functionality to clone an autonomous while also creating a copy of the paths and linked waypoints used in them. This has proven to be a problem with our current PathPlanner workflow.

We created all of our autonomous' on the blue side of the field in our shop, and at competition we tune both sides until there is a clear difference in the field markings or terrain, where we then create a copy to tune for the red side of the field.

This tool was created to simplify the process of fully cloning an autonomous, as it would take much longer to do it by hand. This tool is by no means an example of clean coding practices and was made in 20 minutes. Other than that, it works!

This was programmed in Node.js and can be ran by running ``node .`` within this directory, given that Node.js is installed.

### Example output

```
[0] Any - Preload Only.auto
[1] Center - Royal Flush.auto
[2] Human - Houston Texas.auto
[3] Source - Duality of Bot.auto
Which Auto would you like to duplicate?
Index: 0
```