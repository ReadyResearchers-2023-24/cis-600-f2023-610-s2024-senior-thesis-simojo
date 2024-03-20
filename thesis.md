<!-- # Template description

## Citations and references

## Labeling figures

```markdown
![Label](images/IMAGE_NAME.png)
```

## Labeling tables

To provide a label for a table, write a short caption for the table and prefix the caption
with `Table:` as in the example below:

```
Table: A two-row table demonstrating tables

|Row number | Description |
|:----------|:------------|
|1          |Row 1        |
|2          |Row 2        |
```

## Other template information

Two things specific to this template to also keep in mind:

1. It is your responsibility to remove this description section before building
the PDF version you plan to defend.
2. References _will only appear if cited correctly_ in the text

## Note on `LaTeX` commands

Documents may include specific `LaTeX` commands _in Markdown_. To render these, surround the commands
with markup denoting `LaTeX`. For example:

```
Checkmark character:   $\checkmark$
Superscript character: $^{\dag}$
```

If using a special package not included in the template, add the desired `LaTeX`
package or command/macro to the `header-includes` property in [config.yaml](config.yaml).

Should this package not be included in the environment shipped with this template,
you may also need to add the package to the [GitHub Actions Workflow](.github/workflows/main.yml).

Direct any questions about issues to your first reader.
-->

<!-- From Physics:
Written work in fall semester
Recommended format:
- Introduction and Background: includes previous work in field
- Theory
- Methodology / Experimental Plan
- Preliminary Results
- Future Work
- References: It is expected that the vast majority of references
will be from peer reviewed sources (not Wikipedia!) -->

# Introduction

## Motivation

### The Need for Generalized Autonomous Navigation

Autonomous navigation is necessary for a robotic system to interact with its
surroundings in a real world environment, and it is necessary to realize
technologies such as fully autonomous unmanned aerial vehicles (UAVs) and land
vehicles. Modern robotic systems employ a variety of techniques to achieve
spatial awareness. These systems take the form of ranging sensors (acoustic or
optical) or optical flow, which is a steady stream of camera information that is
used to make assertions about the relative positions of objects. Interpreting
reliable and fast 3D spatial data via optical flow requires extensive training
of a convolutional neural network and large amounts of data. Although recent
work has enabled a racing quadcopter to outperform professional pilots using
optical flow, boasting a speed of $22 \frac{\text{m}}{\text{s}}$, this
approach's success was largely specific to its experimental setup and makes a
weak argument for generalized autonomous navigation [@song2023].

Reinforcement Learning (RL) has proven to be a novel and effective method for
autonomous navigation and control [@gugan2023; @song2023; @doukhi2022]. By
defining a set of desired qualities of a system, the system will learn to
develop its own policy, which is responsible for mapping its instantaneous of
its environment to its action at a given point in time.

### Reinforcement Learning

RL has long been considered to be more adaptive than the industry standard
method of control, PID control, which requires extensive tuning. RL tries to
find the optimal way to map a perceived state to an action by finding what is
called a control policy. The decision making is influenced by a reward factor,
which quantifies the success of the system's actions. A control policy is a set
of rules that define the way in which a system's state is mapped to its next
action. This is similar to the Markov decision process, in which an agent's
action at time $\tau_k+1$ is derived from its state at $\tau_k$. Although a
control policy can be defined qualitatively, it is rather a mapping of a state
tensor to an action tensor, both almost always being multidimensional.

Recent investigations into methods of control for quadcopter systems involve
controlling the quadcopter's *attitude*, or the desired state of its position.
As opposed to PID control, which requires tuning between the feedback loop and
action of the controller, RL autonomously solves control problems by optimizing
its actions with respect to a reward metric. This results in an enhanced ability
to react to diverse situations, which would be considered generalized
intelligence [@bernini2021].

### The Growing UAV Industry

The use of unmanned aerial vehicles (UAVs) or drones is becoming increasingly
ubiquitous across various application domains, including real-time monitoring,
wireless coverage, remote sensing, search and rescue, and delivery of goods.
UAVs are thought of to be especially fit for search and rescue applications,
given the dangerous conditions associated with these operations
[@shakhatreh2019].

Although the FAA states that the rate of increase of recreational UAV owners in
the United States has been reported to be slowing in recent years, the UAV
industry is projected to be worth USD 1.5 trillion by 2040 [@faaForecast, p. 46;
@gugan2023, p. 1].

### The Promise of Ranging Sensors

There are numerous hindrances that pose a barrier to ready, widespread adoption
of UAVs in the industry. UAV cameras are still very prone to over and under
exposure when navigating outdoors. This can interfere with the photogrammetric
processes used by many implementations. *Photogrammetry* refers to the
conversion of a set of 2D images to a 3D model. It can be used in place of
LiDAR or in conjunction with LiDAR. UAV camera systems can be delayed in their
response to higher and lower exposure of the course of a flight because of the
presence of sunlight and shaded areas found outdoors. Further, UAV camera
systems struggle in the presence of precipitation and fog [@gugan2023].

Using ranging sensors can mitigate many of the problems associated with UAVs
that rely on optical flow. Although ranging sensors come in varying forms, they
all rely on measuring the Time of Flight (ToF) of an emitted beam of light, that
is, the difference in time $\Delta t$ between $t_{\text{emitted}}$ and
$t_{\text{received}}$. In robotic applications, manufacturers have opted to use
laser diodes in the near infrared (NIR) band because of their inexpensiveness,
which is a result of the exploding fiber optics industry. Additionally, NIR
light is invisible and less harmful the human eye, giving it credibility in
terms of safety [@raj2020, p. 16].

## Goals of the Project

This project aims to train the COEX Clover quadcopter to perform basic
navigation and obstacle avoidance in randomized scenarios by using an array of
Time of Flight (ToF) sensors. By training the quadcopter to explore randomized
environments, this can demonstrate how using simpler, more economically
affordable sensors can enable a quadcopter to fly in a GPS-denied environment
without the use of LiDAR, which is typically an order of magnitude more
expensive.

## Ethical Implications

### Civilian Use

UAV quadcopters have only recently started being mass produced [@chavez2023]. We
can look at numerous cases over the last decade that signal the importance of
regulating and vetting their use. The implications of autonomous UAVs only
exacerbates this.

The FAA reported that over 1.47 million new recreational drone owners registered
in the United States between December 21, 2015 and the end of December 2022. In
2022, the FAA saw an average of 7,866 newly registered recreational UAV owners
per month [@faaForecast, p. 45]. A higher volume of recreational users implies
the increased risk of misuse across various domains.

Any quadcopter equipped with one or more cameras can be considered a risk to
privacy. A flying system that can be remotely operated has the potential to be
exploited to infringe on privacy. Further, an *autonomous* system could add a
layer of anonymity to enable a malicious party to perform simultaneous
operations for the purpose of infringing on privacy [@cummings2017].

In @cummings2017, the authors gathered five incidents in the past decade that
have marked ethical concerns related to UAV operation, which are listed in
{+@tbl:uavincidents}. These cases provide a mere glimpse of the potential misuse
for UAV technology.

Table: Examples of Incidents that Highlight the Safety, Ethical, and Privacy
Concerns Related to UAV Operation (Source: @cummings2017). {#tbl:uavincidents}

+--------------------------+-------------------------+-------------------------+
| Incident(s)              | Significance            | Source                  |
+==========================+=========================+=========================+
| UAV crashes in           | UAV operator claimed    | [@martin2014]           |
| an Australian triathlon  | someone hacked the      |                         |
| injuring an athlete.     | device resulting in the |                         |
|                          | crash.                  |                         |
+--------------------------+-------------------------+-------------------------+
| Drone carrying Albanian  | Incident viewed as      | [@thetelegraph2014]     |
| flag sparked brawl       | “political provocation” |                         |
| between Serbian and      | by Serbian Foreign      |                         |
| Albanian players.        | Minister, reopening old |                         |
|                          | tensions.               |                         |
+--------------------------+-------------------------+-------------------------+
| FAA reports an           | UAVs encroaching on     | [@jansen2015]           |
| increasing number of     | commercial airspace,    |                         |
| UAVs being sighted by    | increasing safety       |                         |
| commercial airlines      | concerns for commercial |                         |
|                          | airlines and passengers.|                         |
+--------------------------+-------------------------+-------------------------+
| UAV on the White House   | Breach of national      | [@berman2015]           |
| Lawn                     | security                |                         | 
|                          |                         |                         |
+--------------------------+-------------------------+-------------------------+
| UAV carrying radioactive | UAVs used to make a     | [@anderson2015]         |
| materials lands on       | political point in      |                         |
| Japanese Prime           | response to Japan’s     |                         |
| Minister’s Office        | damaged nuclear         |                         |
|                          | reactor                 |                         |
|                          |                         |                         |
+--------------------------+-------------------------+-------------------------+

### Military Use

#### Recreational UAVs in The Russian War on Ukraine

Within the first day of the Russian war on Ukraine, the Ukrainian government
urged Ukrainian citizens on social media to donate their recreational drones to
aid in the effort of defense. Ukraine's tactical use of commercial drones caught
Russia off guard and successfully interfered with and surveilled Russian
soldiers through strikes and reconnaissance missions. Interviews with Russian
soldiers have confirmed the psychological exhaustion they experienced from the
fleets of commercial drones, and Russia later took action to reinforce their own
approach by incorporating quadcopters into their tactics, emulating their
opponent [@chavez2023]. This alarmingly rapid adoption of UAVs for use in
warfare could amplify the aim of other terroristic or violent organizations to
do the same, given the accessibility of this kind of UAV technology.

#### Autonomous UAVs for War

Although using commercial UAVs for military purposes may be novel, the use of
UAVs in armed forces is commonplace today. Military grade UAVs are capable of
long-ranged remote reconnaissance and are often weaponized, which minimizes risk
to the party that uses it.

One may think that this undoubtedly removes a drone operator from the
psychological harm that comes with remorse; however, in a comparison of PTSD
experienced between pilots of manned aircraft and UAV pilots, Johnston presses
evidence that ``physical safety does not translate neatly to psychological
safety" [@johnston2022]. UAV pilots are tasked with hundreds of hours of screen
time, learning how their target behaves through painstaking surveillance. As
opposed to the pilot of a manned aircraft, a UAV pilot may not leave the scene
of an operation in a physical sense. Rather than pinning the emotions and
remorse in a physical location and abandoning it, a UAV pilot is often left with
a psychological landscape from which they are never freed. Studies show that
this effect can lead to PTSD comparable to that of a manned aircraft pilot
[@johnston2022].

The downsides to piloted aerial offense form a powerful argument for investing
in the autonomous equivalent. With the advent of autonomous UAVs, many
governments have invested in the promise that autonomy adds to UAV applications.
Autonomous UAV systems need not maintain a wireless connection between their
on-board control system and a ground control station, which is favorable for
military use because of its resilience to jamming [@khalil2022].

The ethical question formed by the notion of autonomous UAVs for military use is
one that seeks the role of a human in warfare. The prospect of applying
autonomous UAVs for tactical purposes separates military powers from the
emotional and psychological consequences of warfare. If, by removing the
inhibitions that are invoked by considering the humanity of the opponent, the
consequences of tactical strikes and surveillance are completely mitigated. This
could change the paradigm of war and perhaps lead to military powers behaving in
a manner of detachment.

### The Ethics of This Project

The training done in this project does not involve interaction with human
beings, and the reward metric is intended to promote the stochastic (or
naturally random) navigation of an environment. Moreover, there is no camera
used in this project, which reduces the ability of the quadcopter to detect a
human being.

While the narrow scope of this project excludes many ethical conversations that
apply to other UAV systems, it does not completely exempt it from potential
misuse. Any malicious entity could integrate the work done here into an effort
intended to cause harm, which is a side effect of creating a generalizable
system such as the one described in this project.

This project has endless implications in search and rescue, where a quadcopter
could explore an area too dangerous for humans. A quadcopter capable of
navigating any kind of environment could automate many tedious tasks such as
cave surveys, agricultural or residential land surveys, and delivery services.
Generalized navigation is a key component to granting this ability. The
realization of this technology would promote the well-being of a society,
regardless of socioeconomic or cultural factors.

# Related work

#### GPS-Denied Positioning Using ArUco Markers

<!-- related-work -->
In an effort to demonstrate the ability of a quadcopter to perform basic
navigation, the authors in @bogatov2021 used a grid of ArUco markers to provide
the COEX CLover 4 quadcopter with an optical point of reference. They state: "An
ArUco marker is a synthetic square marker composed by a wide black border and
[an] inner binary matrix which determines its identifier (id)." Using its
on-board camera, the drone resolved its own position by comparing it relative to
each ArUco marker.

By default, the ROS module `aruco_detect` for the Clover 4 is capable of
publishing the positions of ArUco markers as *TF frames*, which is a data type
ROS uses to standardize multiple frames of reference in the context of a global
frame of reference [@clover]. With the goal of tracing out four characters along
an invisible plane using the drone's motion, the group demonstrated that their
method of using ArUco markers kept the quadcopter's position within 0.1 to 0.2m
of the desired waypoints, on average. They attribute the cause of the error to
their haphazard positioning of the ArUco markers and improperly defined PID
controller values [@bogatov2021].

While our project may not use ArUco markers, a comparison can be made
between the effectiveness of ArUco markers and an array of ToF sensors for
determining the local position of the quadcopter.

#### Simulating Quadcopter Dynamics

In order for the control loop to properly decide a course of action based on its
inertial measurements, the dynamics of the system must be calculated. This can
be accomplished via an external frame of reference or by on-board sensors.

<!-- related-work -->

The authors of [@de2014] present a "black box" approach to estimating the
quadcopter's state. Rather than meticulously modeling every part of the
quadcopter's dynamics, a general structure of its mechanics was developed. By
simulating the known properties of the quadcopter, such as the positioning of
its propellers and overall mass, the authors roughly defined how the inputs
(roll, pitch, yaw, and thrust) would be transformed by the flight controller.
This method was combined with a Gaussian Process model, which needed to first be
trained on the former model of estimation. After being trained to predict the
quadcopter's real-time movements within minimal error, the Gaussian Process
model was added to the simulation environment and used in the quadcopter's
on-board state estimation. Although the authors were limited by the processing
power required by Gaussian Process models, their results demonstrated how one
can tailor a rough model to fit real world results by adding a probabilistic
correction [@de2014].

### Reinforcement Learning for Robotics

<!-- related-work -->
<!-- FIXME: reference narvekar2020. it is a review article -->

#### Using Reinforcement Learning for Path Planning

<!-- related-work -->

Algorithms such as the famous $\text{A}^{*}$ algorithm require a robotic system
to have a comprehensive understanding of the environment it is in. The authors
in [@hodge2021] combined uses a Proximal Policy Optimization (PPO) algorithm
combined with RL and long short-term memory neural networks for generalized path
planning based only on a systems knowledge of its local surroundings. PPO is
capable of reducing the chances of falling into *local minima* (a common problem
in machine learning when the learning algorithm settles on a sub-optimal
solution before finding the optimal solution) by ensuring that an update to the
learned policy reduces the cost function and does not deviate severely from the
previous policy. By ignoring the altitude of the quadcopter, the authors of
[@hodge2021] reduce the problem to two dimensions, and they leave all matters of
dynamic control up to the reader. This means that their method provides a way to
recommend which movements to make without actually controlling the system.

Our project seeks to both provide a mode of dynamic control and navigation
for the quadcopter system, but we can still compare the accuracy of our
navigation algorithm to that of the PPO-based project in [@hodge2021].

<!-- related-work -->
The authors in [@doukhi2022] present a hybrid approach of dynamic control and
navigation by allowing the quadcopter to act upon its planned motion via
grid-like movements until it approaches any obstacles. Once an obstacle is
approached, the control is handed over to a policy whose goal is to navigate
around the obstacle while maintaining forward motion. For each instantaneous
reward, if there are no obstacles within 2m of the quadcopter, the quadcopter is
greatly rewarded for forward movement. This incentivizes it to continue
exploration. This effort validates the hybrid use of inertial navigation for the
majority of a quadcopter's movements and a handover of control to a trained obstacle
avoidance model for more precise movements.

The approach found in [@doukhi2022] makes a strong argument for hybrid control
because of its refined scope. This will consequently allow for RL training to
take place for a consistent kind of problem, rather than leaving both 'free'
navigation and obstacle avoidance for the quadcopter to handle.

#### Using Reinforcement Learning for Attitude Control

Giving a quadcopter complete control over its attitude requires extensive
training and computational power to train because of how large the action and
state spaces become. Continuous action and state spaces, as opposed to discrete
action and state spaces, require a completely distinct set of algorithms for
training, because of the infinite number of states and actions possible.

<!-- related-work -->
The authors in [@bernini2021] compare different RL algorithms' effectiveness in
controlling a quadcopter's attitude.
<!-- FIXME: more here -->

# Method of approach

This project uses the Copter Express (COEX) Clover quadcopter platform, equipped
with Time of Flight (ToF) ranging sensors, and applies a Deep Deterministic
Policy Gradient (DDPG) algorithm to train it for autonomous navigation. By
simulating the Clover in the Gazebo simulation environment, hundreds of
iterative attempts can be taken to safely train the quadcopter's navigation
ability [@gazebo].

Rather than developing an algorithm for navigation, we employ a variety of RL
techniques to motivate the quadcopter to autonomously explore its environment.
By adding Ornstein-Uhlenbeck noise to each sampled action, we can ensure the
quadcopter's likeliness of navigating its environment.

The code and documentation for running this project can be found at
\url{https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact}.

## Project Design

### COEX Clover Quadcopter Platform

The COEX Clover quadcopter, depicted in {+@fig:clover}, is a platform for
education and research developed by Copter Express. On board, the Clover has the
Raspberry Pi 4 computer for performing computations and running the Robotic
Operation System (ROS). In this specific project's use case, ROS is responsible
for communicating between the on-board flight controller's measurements, which
defines the state, and mapping the state to govern the next action the flight
controller will take. The on-board flight controller is the COEX Pix, which
operates off of the PX4 flight stack, an open source autopilot software for
various applications. The COEX Pix has two built in sensors listed in
{+@tbl:coexpixsensors} [@clover].

![COEX Clover 4 Quadcopter [@clover].](images/clover.png){#fig:clover width=75%}

Table: Built-in sensors in the COEX Pix platform [@clover].
{#tbl:coexpixsensors}

+--------------+--------------------------------------+
| Sensor Name  | Description                          |
+==============+======================================+
| MPU9250 9DOF | accelerometer/gyroscope/magnetometer |
+--------------+--------------------------------------+
| MS5607       | barometer                            |
+--------------+--------------------------------------+

### Time of Flight (Tof) Ranging Sensors

In order for the quadcopter to detect its surroundings, we employ ten Time of
Flight (ToF) ranging sensors, positioned in an arrangement inspired from
[@hodge2021], where the authors propose an octagonal arrangement of of sensors
to measure spatial data laterally along the $x-y$ plane. In this project,
however, we add two more sensors, one facing the $+z$ direction and another
facing the $-z$ direction. By adding these vertical sensors, the quadcopter may
modify its vertical motion to better navigate.

The ToF sensor used in this project is the Adafruit VL53L4CX, depicted in
{+@fig:vl53l4cx}, which has a range from 1mm up to 6m. This device has a FOV of
$18^\circ$ and is controllable via the I2C protocol.

![Adafruit VL53L4CX ToF Distance Sensor.](images/vl53l4cx.jpg){#fig:vl53l4cx
width=75%}

In order to mount the ToF sensors to the Clover, a custom fixture was created to
mount onto the clover. For the eight horizontal ToF sensors, an octagonal
drum-like fixture was developed, which points each ToF sensor at incrementing
angles of $45^\circ$; however, in order for the Clover's legs to not obstruct
the ranging sensors, the ToF sensor drum's yaw is offset by $22.5^\circ$.
{+@fig:tof_sensor_drum} depicts a model of the ToF sensor drum and a top plate
used to mount the sensor in the $+z$ direction. Note that the bottom of the ToF
sensor drum can also mount a ToF sensor. In
{+@fig:tof_sensor_drum_on_clover_in_gazebo}, the simulated model is depicted,
attached to the simulated model of the Clover, along with the visualization of
each link's collision box.

![The ToF sensor drum fixture and one of its base plates rendered in OnShape. The ToF sensor drum mounts to the bottom of the quadcopter, while the single base plate mounts to the top using standoff mounts above the battery. While the physical realization of this has not yet been completed, its simulated equivalent exists.](images/tof_sensor_drum.png){#fig:tof_sensor_drum width=75%}

![(a) The ToF sensor drum fixture mounted to the Clover in Gazebo. Because the top base plate has little effect on collision boxes, it was omitted from the simulation for the sake of brevity. (b) Collision boxes of the Clover model depicted in Gazebo.](images/tof_sensor_drum_on_clover_in_gazebo.png){#fig:tof_sensor_drum_on_clover_in_gazebo width=75%}

<!-- FIXME: add image of collision boxes -->

### Deep Deterministic Policy Gradient (DDPG) Algorithm

#### Gradient Ascent and Descent

The method of learning used in this project is the Deep Deterministic Policy
Gradient algorithm, which maps a continuous state space to a continuous action
space.

In RL algorithms, training is accomplished by giving the system a feedback
mechanism called a reward. The reward may be based off of historical data or on
the most recent state of the system. In DDPG, we train two networks, an
action-value or critic network, and a policy or actor network. The action-value
network has access to historical training data, while the policy network does
not. The two networks inform each other in the training process.

One method of finding an optimal policy for controlling a system is to track the
gradient of the expected reward, which is done by varying the weights of the
policy and measuring the gradient of the reward with respect to the weights. The
training algorithm uses *gradient ascent* in this case, where it tries to
maximize the reward.

In regards to the action-value network, the gradient of the error between the
expected and actual reward with respect to the weights is calculated. The
training algorithm uses *gradient descent* in this latter case, trying to
minimize the error to achieve {+@eq:bellman}

<!-- FIXME: reward metric to actually get the thing to perform navigation has
yet to be determined -->

#### State and Action Spaces

We define the state space $S$ of the quadcopter by twenty-five parameters and
their corresponding range of values, detailed in {+@tbl:state}. Additionally, we
define the action space $A$ of the quadcopter by three parameters and their
corresponding range of values and activation functions, detailed in
{+@tbl:action}.

Table: The values that exist in the state space of the quadcopter system.
{#tbl:state}

+----------------------+----------------------+---------------------------+--------------------------+
| Name                 | Domain               | Units                     | Description              |
+======================+======================+===========================+==========================+
| $x_{\text{desired}}$ | $\mathbb{R}$         | $\text{m}$                | Desired position         |
+----------------------+----------------------+---------------------------+                          +
| $y_{\text{desired}}$ | $\mathbb{R}$         | $\text{m}$                |                          |
+======================+======================+===========================+==========================+
| $p_x$                | $\mathbb{R}$         | $\text{m}$                | Current position         |
+----------------------+----------------------+---------------------------+                          +
| $p_y$                | $\mathbb{R}$         | $\text{m}$                |                          |
+----------------------+----------------------+---------------------------+                          +
| $p_z$                | $\mathbb{R}$         | $\text{m}$                |                          |
+======================+======================+===========================+==========================+
| $q_x$                | $\mathbb{R}$         | dimensionless             | Quaternion orientation   |
+----------------------+----------------------+---------------------------+                          +
| $q_y$                | $\mathbb{R}$         | dimensionless             |                          |
+----------------------+----------------------+---------------------------+                          +
| $q_z$                | $\mathbb{R}$         | dimensionless             |                          |
+----------------------+----------------------+---------------------------+                          +
| $q_{\omega}$         | $\mathbb{R}$         | dimensionless             |                          |
+======================+======================+===========================+==========================+
| $v_x$                | $\mathbb{R}$         | $\text{m}\text{s}^{-1}$   | Current velocity         |
+----------------------+----------------------+---------------------------+                          +
| $v_y$                | $\mathbb{R}$         | $\text{m}\text{s}^{-1}$   |                          |
+----------------------+----------------------+---------------------------+                          +
| $v_z$                | $\mathbb{R}$         | $\text{m}\text{s}^{-1}$   |                          |
+======================+======================+===========================+==========================+
| $\omega_x$           | $\mathbb{R}$         | $\text{rad}\text{s}^{-1}$ | Current angular velocity |
+----------------------+----------------------+---------------------------+                          +
| $\omega_y$           | $\mathbb{R}$         | $\text{rad}\text{s}^{-1}$ |                          |
+----------------------+----------------------+---------------------------+                          +
| $\omega_z$           | $\mathbb{R}$         | $\text{rad}\text{s}^{-1}$ |                          |
+======================+======================+===========================+==========================+
| $\text{range}_0$     | $[0.001, 6]$         | $\text{m}$                | Distance measurements    |
+----------------------+----------------------+---------------------------+ from ToF sensors         +
| $\text{range}_1$     | $[0.001, 6]$         | $\text{m}$                |                          |
+----------------------+----------------------+---------------------------+                          +
| $\text{range}_2$     | $[0.001, 6]$         | $\text{m}$                |                          |
+----------------------+----------------------+---------------------------+                          +
| $\text{range}_3$     | $[0.001, 6]$         | $\text{m}$                |                          |
+----------------------+----------------------+---------------------------+                          +
| $\text{range}_4$     | $[0.001, 6]$         | $\text{m}$                |                          |
+----------------------+----------------------+---------------------------+                          +
| $\text{range}_5$     | $[0.001, 6]$         | $\text{m}$                |                          |
+----------------------+----------------------+---------------------------+                          +
| $\text{range}_6$     | $[0.001, 6]$         | $\text{m}$                |                          |
+----------------------+----------------------+---------------------------+                          +
| $\text{range}_7$     | $[0.001, 6]$         | $\text{m}$                |                          |
+----------------------+----------------------+---------------------------+                          +
| $\text{range}_8$     | $[0.001, 6]$         | $\text{m}$                |                          |
+----------------------+----------------------+---------------------------+                          +
| $\text{range}_9$     | $[0.001, 6]$         | $\text{m}$                |                          |
+======================+======================+===========================+==========================+

Table: The values that exist in the action space of the quadcopter system and
their corresponding activation functions.
{#tbl:action}

+--------+----------------------+---------------+---------------------+
| Name   | Domain               | Units         | Activation Function |
+========+======================+===============+=====================+
| r      | $[0.2, 6]$           | meters        | RelU                |
+--------+----------------------+---------------+---------------------+
| theta  | $[0, \pi]$           | radians       | RelU                |
+--------+----------------------+---------------+---------------------+
| phi    | $[0, 2\pi]$          | radians       | RelU                |
+--------+----------------------+---------------+---------------------+

### Gazebo Simulation Environment

This project uses Gazebo to run a simulated environment of the Clover. Gazebo is
an open source tool for simulating robotics; it simulates the dynamics and
actuation of robotic systems. COEX has developed a simulation environment that
can be used in Gazebo for simulating the Clover [@gazebo].

Gazebo allows for the environment to be defined by physical parameters such as
wind, atmospheric type, and gravity, and it uses the *Open Dynamics Engine* to
resolve physical interactions.

#### `.world`, `.sdf`, `.urdf` Files and Meshes

Gazebo supports the parsing of `.world` files, which is a special kind of `.sdf`
file that describes the environment to Gazebo. `.sdf` stands for Simulation
Description Format, and it defines a specification for describing elements in
simulation environments. The specification (\url{http://sdformat.org/spec})
allows for `<world>`, `<model>`, `<actor>`, and `<light>` root elements.

<!-- FIXME: is this reference formatted correctly? Check after compiling. -->

COEX has supplied their own `.sdf` and `.urdf` (Universal Robotics Description
Format) files that describe the Clover. These files are located within the
`clover_description` package; however, they are written as `.xacro` files, which
is an intermediate format used to construct larger hierarchies of `.sdf` and
`.urdf` files.

##### Links and Joints

Both `.sdf` and `.urdf` allow for the specification of *links*, parts of a
robotic system, and *joints*, hierarchical relative frames of reference between
links. In general, the fundamental link that comprises every robotic system is
named, `base_link`. An example of a file that describes a link connected to base
link is seen in the following code block:

```xml
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- macro name to be referenced in other files -->
  <xacro:macro name="clover4_base_macro"
    params="mass body_width body_height *inertia">
    <link name="base_link"></link>
    <!-- join base_link to its inertial properties -->
    <joint name="base_joint" type="fixed">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <parent link="base_link" />
      <child link="base_link_inertia" />
    </joint>

    <link name="base_link_inertia">
      <!--
        define inertial elements for
        calculating dynamics via ode
      -->
      <inertial>
        <mass value="${mass}" />  <!-- [kg] -->
        <origin xyz="0 0 0" />
        <xacro:insert_block name="inertia" />
      </inertial>
      <!--
        define visual element,
        has no effect on physics
      -->
      <visual name="body">
        <origin xyz="0 0 0.025" rpy="0 0 0" />
        <geometry>
          <!--
            Note: Texture files are expected
            to be in the same directory as the model
          -->
          <mesh filename="package://clover_description/meshes/clover4/clover_body_solid.dae"
            scale="1 1 1"/>
        </geometry>
      </visual>
      <!--
        define collision element for
        calculating collisions via ode
      -->
      <collision name="base_link_inertia_collision">
        <origin xyz="0 0 0.05" rpy="0 0 0" />
        <geometry>
          <box size="${body_width} ${body_width} ${body_height}" /> <!-- [m] [m] [m] -->
        </geometry>
      </collision>
    </link>

    <!-- tell gazebo about this link, specifying custom physics properties -->
    <gazebo reference="base_link">
      <self_collide>0</self_collide>
    </gazebo>
    ...
  </xaro:macro>
</robot>
```

##### Custom Mesh

See [](#sec:preparing-stl-files-for-simulation) for information on how
to prepare `.STL` files for simulations.

<!-- FIXME: explain my ToF sensor drum -->

<!-- more information about .world files, .sdf files, and .xacro files -->

## Experimental Design

In this project, we employ the 

### World Generation

In order to test the robustness of our training algorithm, we use the package
`pcg_gazebo` by Bosch Research to generate environments to test and train the
Clover in [@manhaes2024]. For the experiment, ten procedurally-generated
`.world` files were created.

Each world is generated by taking the union of the footprints of $n$ number of
rectangles. Each generated world has an increasing number of rectangles used in
order to make the *cirriculum* increasingly difficult. {+@fig:footprint} depicts
an example of a `.world` file generated using `pcg_gazebo`.

[A world generated using `pcg_gazebo` with the Clover located at the origin. Each radial blue line coming from the Clover is its ranging sensor's feedback, which is evidently incident on the light pink cylinder in this image.](images/footprint.png) {#fig:footprint}

<!-- FIXME: now talk about randomly putting objects and finding free spots -->
<!-- FIXME: make sure to reference the appendix for tutorials on how to generate
worlds. -->

### Training Steps

For each *episode*, or iteration, of the training process, a state $s$ is
derived from the quadcopter's telemetry. The state is then provided as an input
to the policy, which samples a reward and applies Ornstein-Uhlenbeck (OU) noise
to encourage exploration without disrupting the continuum of the actions being
taken [@spinningup2018]. After the policy provides an action, the action is
converted from spherical to Cartesian coordinates and given to the PX4 flight
controller as a setpoint.

The Clover ROS package provides the service `navigate` that uses the PX4's
on-board Visual Position Estimation (VPE) to move to given coordinates. By
connecting a RPi camera and a downward-facing ToF ranging sensor to a Raspberry
Pi running `mavros` connected to the PX4, the PX4 can calculate its local
position [@clover].

<!-- FIXME: need to explain how we mitigated timing issues with repeatable discrete
time steps -->

<!-- FIXME: explain how I ended up choosing to do a navigate_wait approach and
why and how it mitigates issues with setting the quadcopter attitude -->

After $a$ is acted upon, $s$ is read once again in order to determine the reward
metric, which takes off points proportional to the distance away the quadcopter
is from its goal or if the quadcopter has crashed or flipped over.

The reward metric is then recorded, and the gradient of the reward metric is
measured with respect to the action and state variables [@spinningup2018]. The
*Adam* optimizing function, provided by TensorFlow, is then used to modify the
policy [@tensorflow]. This is when the learning happens.

## Theory

### Deep Reinforcement Learning

As stated, this project uses a Deep RL algorithm known as the Deep Deterministic
Policy Gradient (DDPG) algorithm. In Deep RL, an agent transforms its
observation or state of its environment to an action which it takes on its
environment ^[To be pedantic, a state describes the comprehensive state of the
agent's environment, omitting no information, while an observation may be a
partial interpretation of the environment; however, we will refer to the
system's state as a state $s$ for simplicity sake.]. After doing so, the agent's
action is associated with a reward value. Thus, for each step in time $t$, the
agent has a state $s_t$ and reward $r_t$.

Most agents are defined by their policy, which is a relation that takes $s_t$
and returns $a_t$:

$$
a_t = \mu(s_t)\ \text{or}\ a_t \sim \pi_\theta(\cdot | s_t),
$$

with $\theta$ referring to the policy's parameters or weights, assuming it is a
neural network. $\mu$ is used to refer to a deterministic policy, while $\pi$
refers to a stochastic policy.

Every Deep RL algorithm seeks to approximate the optimal policy $\mu^*(s)$ or
$\pi^*(s)$. The optimal policy is the policy that maximizes the expected
cumulative reward over a sequence of state-action pairs.

Some Deep RL algorithms use an action-value function $Q(s,a)$ which gives an
expected reward value from taking action $a$ given state $s$ [@zhu2021]. In Deep
Q Learning, for example, the policy is calculated by approximating the optimal
action-value function, denoted as $Q^*(s,a)$. This is because the optimal policy
can be composed of the optimal action-value function by taking the action $a$
that maximizes $Q^*(s,a)$:

$$
\pi^*(s) = \arg\max_a Q^*(s,a).
$$

There are many ways in which an agent can learn an approximation of the optimal
policy, which is what makes Deep RL such a vast field. In this project, we
specifically explore the Deep Deterministic Policy Gradient (DDPG) algorithm
[@spinningup2018].

### Deep Deterministic Policy Gradient (DDPG) Algorithm

The DDPG algorithm is a kind of Deep RL algorithm that simultaneously learns an
action-value function $Q(s,a)$ and a policy $\mu(s)$. Both sides of the learning
process inform each other, leading to enhanced, albeit more prolonged, results
[@zhu2021].

To approximate the optimal action-value function, the DDPG algorithm tries to
approximate the Bellman equation which represents the optimal action-value
function:

$$
Q^*(s,a) = E_{s' \sim P}\Big[r(s,a) + \gamma \max_{a'} Q^* (s', a')\Big].
$$ {#eq:bellman}

{+@eq:bellman} describes the expected value ($E_{s' \sim P}[...]$) given a state
$s'$ sampled from a distribution $P$ of the current reward $r(s,a)$ and
discounted future reward $\gamma \max_{a'} Q^* (s', a')$, where $\gamma$
is the discount factor. The discount factor weighs near-future rewards more than
distant-future rewards.

When approximating the optimal action-value function, which is, in this case,
achieved via a neural network parameterized by its weights, the goal is to
minimize the expectation value of the square of the difference between both
sides of the Bellman equation. Ideally, this difference would be zero, which is
why this algorithm tries to minimize the said difference.

In practice, performing training off a recursive function is unstable, so a
*target network* is used. The weights of *target networks* are only periodically
copied over from the main network, and the optimal action-value function is
approximated by minimizing the difference between the target network's output
and the main network's output using stochastic gradient descent.

Approximating the optimal policy in DDPG is done by performing gradient ascent
on $Q_\phi(s, \mu_\theta(s))$ with respect to $\theta$, where $\phi$ represents
the weights of action-value network. We use $\mu$ as the name of the policy
function, indicating that it is deterministic of state. The subscript $\theta$
refers to its parameters or weights, assuming it is a neural network
[@spinningup2018]. Ultimately, learning the optimal policy tries to solve
{+@eq:policy}:

$$
\max_\theta E_{s\sim \mathcal{D}}\Big[ Q_\phi(s,\mu_\theta(s)) \Big].
$$ {#eq:policy}

### Quadcopter Dynamics

#### Rotational Matrix Defined by Roll, Pitch, and Yaw

With the earth's reference frame as $R^{E}$ and the quadcopter's body's
reference frame as $R^{b}$, the *attitude* of the quadcopter is known by the
orientation of $R^{b}$ with respect to $R^{E}$. We determine this from the
rotational matrix defined in {+@eq:rotationalmatrix} [@doukhi2022].

$$
\begin{bmatrix}
\cos \phi \cos \theta & \sin \phi \sin \theta \cos \psi - \sin \psi \cos \phi & \cos \phi \sin \theta \cos \psi + \sin \psi \sin \phi \\
\sin \phi \cos \theta & \sin \phi \sin \theta \sin \psi + \cos \psi \cos \theta & \cos \phi \sin \theta \sin \psi - \sin \phi \cos \psi \\
-\sin\theta & \sin \phi \cos \theta & \cos \phi \cos \theta
\end{bmatrix}
$$ {#eq:rotationalmatrix}

The roll, pitch, and yaw of the quadcopter are its angular orientations around
the $x$, $y$, and $z$ axes respectively, with positive rotations following a
right-handed rotation around each axes. In {+@eq:rotationalmatrix}, these angles
are represented by
$\phi$, $\theta$, and $\psi$.
Rotational matrices can rotate a vector around the origin. If, for example, we
begin with $\vec{v} = \begin{bmatrix}1 \\ 1 \\ 1\end{bmatrix}$, ${\vec{v}\,}'$, the
rotated vector, can be calculated by multiplying the rotational matrix on
$\vec{v}$.

Suppose we had roll, pitch, and yaw values of
$\phi = \frac{\pi}{2}$, $\theta = \frac{\pi}{2}$, and $\psi = \frac{\pi}{2}$.
Then, $\vec{v}$ would become:

$$
{\vec{v}\,}' =
\begin{bmatrix}
0 \cdot 0 & 1 \cdot 1 \cdot 0 - 1 \cdot 0 & 0 \cdot 1 \cdot 0 + 1 \cdot 1 \\
1 \cdot 0 & 1 \cdot 1 \cdot 1 + 0 \cdot 0 & 0 \cdot 1 \cdot 1 - 1 \cdot 0 \\
-1 & 1 \cdot 0 & 0 \cdot 0
\end{bmatrix}
\cdot
\begin{bmatrix}1 \\ 1 \\ 1\end{bmatrix}
$$

$$
= \begin{bmatrix}
1\cdot (0 \cdot 0) + 1\cdot (1 \cdot 1 \cdot 0 - 1 \cdot 0) + 1 \cdot (0 \cdot 1 \cdot 0 + 1 \cdot 1) \\
1\cdot (1 \cdot 0) + 1\cdot (1 \cdot 1 \cdot 1 + 0 \cdot 0) + 1 \cdot (0 \cdot 1 \cdot 1 - 1 \cdot 0) \\
1\cdot (-1) + 1\cdot (1 \cdot 0) + 1 \cdot (0 \cdot 0)
\end{bmatrix}
= \begin{bmatrix}
1 \\
1 \\
-1
\end{bmatrix}.
$$

#### Inertia Tensor

The inertia tensor of a system is a $3 \times 3$ matrix $I$ that determines the
resistance of an object to rotational motion. Its heavy use in this project's
source code and robotics in general demands its explanation and derivation. We
can derive it by starting from the basic definition of inertia. Note that this
derivation will largely rely on the derivation found in [@fowles2005].

Consider a rigid body rotating around an axis in the direction of a unit vector
$\vec{n}$. If we consider this rigid body to be made up of tiny particles of
mass $m_i$, taking the sum of each of their masses times the square of their
distance perpendicular to the axis of rotation yields the moment of inertia.

$$I = \sum_i m_i r_{\perp i}^2.$$ {#eq:sum_of_mass_times_r}

If $\vec{r}_i$ is the distance between the origin and the particle, then
$r_{\perp i} = |\vec{r}_i| \sin{\theta_i}$, where $\theta_i$ is the angle
between $\vec{r}_i$ and $\vec{n}$. Because
$|\vec{r}_i \times \vec{n}| = |\vec{r}_i||\vec{n}|\sin{\theta_i}$ and
$|\vec{n}| = 1$,

$$
r_{\perp i} = |\vec{r}_i|\sin{\theta} = |\vec{r}_i \times \vec{n}|.
$$ {#eq:inertial_tensor_cross_product}

Because $\vec{n}$ is a unit vector, we can represent it by its direction
cosines:

$$
\vec{n} = \begin{bmatrix}
\cos{\alpha} \\
\cos{\beta} \\
\cos{\gamma}
\end{bmatrix},
$$

and, letting $\vec{r}_i = \begin{bmatrix}x_i \\ y_i \\ z_i\end{bmatrix}$ define
the position of our point particle, we have

$$
\begin{array}{rcl}
r_{\perp i}^2 & = & |\vec{r}_i \times \vec{n}| \\
& = & \left(y_i \cos{\gamma} - z_i \cos{\beta}\right)^2
    + \left(z_i\cos{\alpha} - x_i \cos{\gamma}\right)^2
    + \left(x_i \cos{\beta} - y_i \cos{\alpha}\right)^2
\end{array}
$$

$$
\begin{array}{rcl}
r_{\perp i}^2 & = &   \left(y_i^2 + z_i^2\right)\cos^2 \alpha
                    + \left(z_i^2 + x_i^2\right)\cos^2 \beta
                    + \left(x_i^2 + y_i^2\right)\cos^2 \gamma \\
              &   & - 2 y_i z_i \cos{\beta}\cos{\gamma}
                    - 2 z_i x_i \cos{\gamma}\cos{\alpha}
                    - 2 x_i y_i \cos{\alpha}\cos{\beta}
\end{array}.
$$ {#eq:r_perp_squared_direction_cosines}

Putting {+@eq:r_perp_squared_direction_cosines} back into
{+@eq:sum_of_mass_times_r},

<!-- FIXME: write equation 9.1.6 from the book and finish it out from there. -->

$$
\begin{array}{rcl}
I & = &   \left(y_i^2 + z_i^2\right)\cos^2 \alpha \\
  &   & + \left(z_i^2 + x_i^2\right)\cos^2 \beta \\
  &   & + \left(x_i^2 + y_i^2\right)\cos^2 \gamma \\
  &   & - 2 y_i z_i \cos{\beta}\cos{\gamma} \\
  &   & - 2 z_i x_i \cos{\gamma}\cos{\alpha} \\
  &   & - 2 x_i y_i \cos{\alpha}\cos{\beta}
\end{array}.
$$

#### Rotational Matrix Defined by Roll, Pitch, and Yaw

We are treating the quadcopter system as a rigid body. Thus, directly using
Newton's Second Law of Motion, we can derive the Newton-Euler formulation for
this system, which gives us the rotational and translational dynamics in
{+@eq:dynamics} [@doukhi2022].

$$
\begin{array}{ll}
\ddot{x} & =
\displaystyle \left(\cos(\phi)\sin(\theta)\cos(\psi)
\displaystyle + \sin(\phi)\sin(\psi)\right) \frac{1}{m}U_1 \\[2ex]
\ddot{y} & =
\displaystyle \left(\cos(\phi)\sin(\theta)\sin(\psi)
\displaystyle - \sin(\phi)\cos(\psi)\right) \frac{1}{m}U_1 \\[2ex]
\ddot{z} & =
\displaystyle -g + \left(\cos(\phi)\cos(\theta)\right) \frac{1}{m} U_1 \\[2ex]
\ddot{\phi} & =
\displaystyle \dot{\theta} \dot{\psi} \left( \frac{I_{yy}
\displaystyle - I_{zz}}{I_{xx}} \right) \frac{1}{I_{xx}} U_2 \\[2ex]
\ddot{\theta} & =
\displaystyle \dot{\phi} \dot{\psi} \left( \frac{I_{zz}
\displaystyle - I_{xx}}{I_{yy}} \right) \frac{1}{I_{yy}} U_3 \\[2ex]
\ddot{\psi} & =
\displaystyle \dot{\theta} \dot{\phi} \left( \frac{I_{xx}
\displaystyle - I_{yy}}{I_{zz}} \right) \frac{1}{I_{zz}} U_4
\end{array}
$$ {#eq:dynamics}

These represent the behavior of the system, given $(U_1, U_2, U_3, U4)$, the
inputs for altitude, roll, pitch, and yaw respectively. $I_{xx}, I_{yy}, and
I_{zz}$ are the moments of inertia along the $x$, $y$, and $z$ axes
[@doukhi2022].

<!-- FIXME: reference paper on Newton-Euler formulation -->
<!-- FIXME: reference de2014 -->

### ToF Ranging Sensors

The ToF sensor used in this project, the Adafruit VL53L4CX, emits 940nm light
from a Vertical Cavity Surface-Emitting Laser Diode (VCSEL). Because of the
novel properties of the VCSEL, it is able to maintain a low operating power.

ToF sensors measure the change in time between the initial emission and final
reception of light, $\Delta t = t_{\text{final}} - t_{\text{initial}}$. Because
$v_\text{air} = \frac{c}{n_\text{air}}$, we know the total distance traveled to
be

$$
\Delta s = \frac{c \Delta t}{2}.
$$

#### Laser Basics

The term 'laser' stands for **l**ight **a**mplification by the **s**timulated
**e**mission of **r**adiation. In the early twentieth century, Einstein proved the
existence of stimulated emission when theorizing the existence of an equilibrium
between light and matter. At the time, stimulated emission had not yet been
discovered, and the only two known interactions between light and matter were
*spontaneous emission* and *absorption*.

In stimulated emission, when an incoming photon with energy $E_1 = h\nu$
interacts with an already-excited quantum system of energy $E_1$, the quantum
system will emit an identical photon with the same direction, polarization,
phase, and momentum as the incoming photon. Thus, the incoming photon
"stimulates" the quantum system to emit an identical photon [@pedrotti1993]. The
result is an coherent duo of photons. This process is what allows lasers to be
coherent light sources.

To explain the existence of stimulated emission, Einstein considered 'matter' to
be a collection of quantum states with $N_2$ states of energy $E_o + h\nu$ and
$N_1$ states of energy $E_o$. Einstein showed that the rate of change of $N_1$
and $N_2$ due to the each of the three different kinds of radiative processes
was proportional to a coefficient that generalized the underlying stochastic
quantum process happening. The coefficients for each of the three processes,
shown in {+@fig:einsteincoefficients}, are now referred to as the Einstein
coefficients.

<!--A21-->
**Spontaneous emission** relies on the $A_{21}$ coefficient:

$$
\left(\frac{dN_2}{dt}\right) = -A_{21}N_2
$$

This implies that the rate of excited states going through spontaneous emission is proportional to the
number of excited systems and the Einstein coefficient $A_{21}$.

<!--B21-->
**Stimulated emission** relies on the $B_{21}$ coefficient:

$$
\left(\frac{dN_2}{dt}\right) = -B_{21}N_2\rho(\nu)
$$

This implies that the rate of excited states going through stimulated emission
is proportional to the number of excited systems, the Einstein coefficient
$B_{21}$, and the density of the radiation field through the matter $\rho(\nu)$.

<!--B12-->
**Absorption** relies on the $B_{12}$ coefficient:

$$
\left(\frac{dN_1}{dt}\right) = -B_{12}N_1\rho(\nu)
$$

This implies that the rate of ground states going through absorption is
proportional to the number of ground state systems, the Einstein coefficient
$B_{12}$, and the density of the radiation field through the matter $\rho(\nu)$.

![The three radiative processes and their corresponding Einstein coefficients
[@pedrotti1993].](images/einsteincoefficients.png){#fig:einsteincoefficients
width=75%}

Einstein's work caused physicists to ponder the applications of such a
phenomenon. In 1954, C. H. Townes leveraged the process of stimulated emission
to create an apparatus for amplifying microwave light, which was named a maser
(**m**icrowave **a**mplifier based on the **s**timulated **e**mission of
**r**adiation). In 1960, T. H. Maiman created the first laser device, which uses
a ruby crystal as its medium [@pedrotti1993, p. 426]. Maiman's device used a
flashlamp to send photons into the ruby medium, which was located at the center
of a Fabry-Perot optical cavity. As the flashlamp 'pumps' photons into the ruby
medium, the $\text{Cr}^{3+}$ ions in the medium are excited to a higher energy
level and then decay to a lower excited state within picoseconds. This lower
excited state has a lifetime of approximately $3\text{ms}$. By continually
pumping the ruby medium with photons, the number of atomic systems in the
excited state, $N_2$, increases such that new photons entering the medium are
most likely to cause stimulated emission. This is known as a population
inversion. By placing the medium in a Fabry-Perot optical cavity, this
encourages photons to be emitted along the optical axis. As an increasing number
of emissions happen along the optical axis, the system reaches a point where all
stimulated emissions are along the optical axis. Because these stimulated
emissions are intrinsically coherent, the result is in-phase, coherent light
resonating along the optical axis. By having one mirror of nearly perfect
reflectivity and another with $\sim 90\text{\%}$ reflectivity, a fraction of the
coherent light is emitted as a columnated beam that we know to be laser light
[@saleh2019, p. 477-478].

Lasers are defined by three fundamental components: an external energy source or
*pump*, an *amplifying medium*, and a *resonator*. The pump adds energy to the
system to achieve a population inversion, which is when $N_2$ passes a threshold
to sustain amplification through stimulated emission. The amplifying medium is
the collection of matter that holds energy and emits photons. The amplifying
medium is chosen based upon its energy levels, which directly determine the
frequency of radiation it is capable of emitting. Lastly, the resonator directs
photons back and forth through the amplifying medium. The most simple form of a
resonator is two precisely aligned mirrors that are placed along the optical
axis. One mirror has the highest reflectivity possible, while the other is given
a reflectivity slightly less than 100% to allow a fraction of the internally
resonating light to be emitted [@pedrotti1993, p. 431-434].

Lasers produce monochromatic, coherent light, which has limitless applications
for the medical field, sensing devices, and aiding our understanding of the
nature of light.

#### The VCSEL

A Vertical Cavity Surface Emitting Laser (VCSEL) (depicted in {+@fig:vcsel}) is a
special kind of diode laser that can be fabricated on the scale of micrometers
through lithography. By depositing the laser cavity in a vertical arrangement,
thousands of VCSELs can be fabricated on a single silicon wafer. Because of
their small size, different architectural considerations must be taken into
account. At this scale, light does not behave classically, and thus, in order to
create the optical cavity with the necessary reflectivity, alternating
semiconductor layers are placed to achieve a near 100% reflectivity. This
configuration is known as a Distributed Bragg Reflector (DBR) [@iga2000].

![A model of a VCSEL on a silicon wafer
[@iga2000].](images/vcsel.png){#fig:vcsel width=75%}

<!-- FIXME: discuss how DBRs work -->
<!-- FIXME: discuss how pumping works -->

VCSELs use less power than traditional lasers because of their dependence on the
energy band gap of their active medium. VCSELs rely on electronic state
transitions for generating photons. At this size, the possible state transitions
are in a continuum rather than discrete, which increases the probability of an
electron transitioning to a lower state. Thus, a higher fraction of pumped
electrons directly contribute to the net output intensity [@iga2000].

VCSELs are ubiquitous in the context of LiDAR mechanisms because of their
inexpensiveness, small size, and ability to transmit continuously [@raj2020].

<!-- FIXME: beef this up for the second semester -->

# Preliminary Results

<!-- FIXME: this is strictly writing for the first semester -->

In the current implementation of this project, the quadcopter is being trained
to hover at a static position near its origin within a virtual machine provided
by COEX. The quadcopter's reward metric, given in {+@eq:reward} is subtracted by
its distance from the desired position of $(x_{\text{desired}},
y_{\text{desired}}, z_{\text{desired}}) = (0\text{m}, 0\text{m}, 1\text{m})$. To
incentivize the quadcopter to hover at $z_{\text{desired}} = 1\text{m}$, a
Gaussian function centered around $z_s = 1\text{m}$ is added to the reward.

$$
\begin{array}{ll}
\text{reward}
\equiv &
100\left(e^{-(z_s - 1)^2} - 1\right) - \\[2ex]
& \sqrt{
    \left(x_s - x_\text{desired}\right)^2
    + \left(y_s - y_\text{desired}\right)^2
    + \left(z_s - z_\text{desired}\right)^2
  } \\
\end{array}
$$ {#eq:reward}

The most prominent challenge with the approach of using Gazebo for simulating
the episodes has been determining a way for the simulation to step through time
at a constant rate. ROS has a callback function capable of measuring the time
elapsed during a single control loop and throttling its speed to maintain a
consistent frequency of reading its state and taking action. This could allow a
predictable control frequency rather than a variable control frequency. A
variable control frequency means that a model may not make appropriate decisions
between control loops. The ability to maintain a consistent timing between each
control iteration would provide much more stability regarding the model's
control of the quadcopter system.

During each episode of training, once the quadcopter flips upside down or flies
out of sight, it is most appropriate to tack an extremely low reward value onto
the data for that episode and reset the simulation environment; however, because
the PX4 flight controller software is not preemptive by default, meaning that it
cannot be easily suspended, the flight controller is not able to handle the
sudden change in position when the quadcopter's position is reset using Gazebo.
Although Gazebo allows for an option to pause the physics engine, the PX4 flight
controller software must be manually modified in order to have this feature.
This feature is not yet a part of the current implementation, and the PX4 still
poses as an issue to switching quickly between episodes.

This issue has prevented the quadcopter system from learning how to control,
because each episode cannot be reset.

# Future Work

## Fixing Timing Inconsistencies in PX4

The next step of this project is to fix the timing inconsistencies of the
simulation such that each control loop, or observation-action pair, happens at a
consistent frequency that can be reproduced in the real world. This may involve
creating a custom version of the PX4 flight controller software capable of
preemptive behavior.

Presumably, fixing the timing inconsistency will allow the quadcopter system to
train itself to hover at a static position. Once this is demonstrated, it will
then be appropriate to explore techniques to train the quadcopter to navigate.

## Simulating and Physically Implementing ToF Sensors

In order for the quadcopter to navigate, it must have the array of ToF sensors.
This means that the ToF must also be simulated in Gazebo with the exact
specifications of the Adafruit VL53L4CX sensors. Additionally, the ten ToF
sensors must be physically mounted to the quadcopter, which will require a 3D
printed fixture. This means that time must be taken to draft, print, and
simulate this design.

## Developing Reward Metrics to Incentivize Quadcopter Navigation

Developing reward metrics to incentivize quadcopter navigation is arguably the
crux of this project, and it will require continuous self-assessment and
reevaluation. By looking upon the work found in [@doukhi2022], we will determine
if certain parts of the quadcopter's navigation are appropriate to hand off to
the inertial navigation process.

# Appendix

## Preparing `.STL` Files for Simulation {#sec:preparing-stl-files-for-simulation}

<!-- FIXME: discuss, once you have an stl file, how to load into meshlab -->
<!-- then discuss how to put into blender to convert to a colladae -->

## Procedurally Generating Rooms Using `pcg` Module and `pcg_gazebo`

In order to test the robustness of a model, it is helpful to evaluate its
performance in random environments. In the Clover VM, this can be done by using
the `pcg_gazebo` package, created by Bosch Research [@manhaes2024]. A wrapper
for this package exists under
\url{https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact} in the
directory \texttt{src/pcg}.

### Using `pcg` for Room Generation

After cloning
\url{https://github.com/ReadyResearchers-2023-24/SimonJonesArtifact} to the
Clover VM, create a Python virtualenv in the `pcg` root directory and install
from `requirements.txt`:

```sh
cd /path/to/SimonJonesArtifact/src/pcg
python3 -m virtualenv venv
pip install -r requirements.txt
```

Now that you have all of the necessary packages, assuming that you have properly
sourced your shell in `SimonJonesArtifact/devel`, you can run the `generate`
script under `pcg`:

```sh
rosrun pcg generate -h
```

If this works correctly, you should see a help output describing the possible
CLI flags. To generate ten worlds, saving them to the `.gazebo/` directory, you
can run the following command:


```sh
rosrun pcg generate \
  --models-dir=/home/clover/.gazebo/models \
  --worlds-dir=/home/clover/.gazebo/worlds \
  --num-worlds=1
```

However, by default, the generated worlds are stored to
`/path/to/SimonJonesArtifact/src/pcg/resources/worlds`, and the models are
stored at `/path/to/SimonJonesArtifact/src/pcg/models`. This allows them to be
incorporated into the project's ROS path by default.

### Installing `pcg_gazebo`

For manually using `pcg_gazebo` without the custom `pcg` module, you must
install `pcg_gazebo`. To install `pcg_gazebo` on the Clover VM, start by
updating the system:

```sh
sudo apt-get update
sudo apt upgrade
```

Then install supporting packages:

```sh
sudo apt install libspatialindex-dev pybind11-dev libgeos-dev
pip install "pyglet<2"
pip install markupsafe==2.0.1
pip install trimesh[easy]==3.16.4
```

Then, you may need to update `pip`, as the version that comes by default in the
VM is not up to date:

```sh
sudo pip install --upgrade pip
```

Then install the `pcg_gazebo` package:

```sh
pip install pcg_gazebo
```

Before running, make sure to create the default directory where the tool will
save the world files, `~/.gazebo/models`:

```sh
mkdir -p ~/.gazebo/models
```

### Running `pcg_gazebo`

For a basic cuboid room, one can run the following:

```sh
pcg-generate-sample-world-with-walls \
  --n-rectangles 1 \
  --world-name <your-world-name> \
  --preview
```

This will generate the world file `~/.gazebo/models/<your-world-name>.world`
that contains a cuboid.

Other examples, that incorporate randomly placed obstacles, are shown in the
following:

```sh
pcg-generate-sample-world-with-walls \
  --n-rectangles 10 \
  --n-cubes 10 \
  --world-name <your-world-name> \
  --preview
pcg-generate-sample-world-with-walls \
  --n-rectangles 10 \
  --x-room-range 6 \
  --y-room-range 6 \
  --n-cubes 15 \
  --wall-height 6 \
  --world-name <your-world-name> \
  --preview
pcg-generate-sample-world-with-walls \
  --n-rectangles 10 \
  --n-cylinders 10 \
  --world-name <your-world-name> \
  --preview
pcg-generate-sample-world-with-walls \
  --n-rectangles 10 \
  --n-spheres 10 \
  --world-name <your-world-name> \
  --preview
```

# References

::: {#refs}
:::
