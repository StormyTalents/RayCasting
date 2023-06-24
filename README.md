# RayCastingSimulation
An AI which simulates a robot that uses AI techniques to perform tasks such as estimating its location on a map and navigating towards a goal through an unknown maze in real time.

![image](https://github.com/StormyTalents/RayCasting/assets/98739389/21525bc8-01c6-4bb5-b040-dc3b6fd1d5c5)

<p>Given an image, the simulation creates a map and uses AI techniques such as ray casting, spatial mapping, Bayesian filtering, and pathfinding to perform tasks such as navigating from a start to a goal given no knowledge of the layout the map (as shown in the image above), or determine its location within the map based on the features of the map it has encountered so far.</p>

https://github.com/StormyTalents/RayCasting/assets/98739389/7918ec36-17e2-4b12-98a2-0777a5a60ad4

<p>The above video shows the AI navigate through the maze from start to goal without any prior knowledge of the map. This is done solely by collecting data from its sensors (the rays are shown in red) which report distances to obstacles, and then using techniques such as spatial mapping and pathfinding (modified A* algorithm).</p>
