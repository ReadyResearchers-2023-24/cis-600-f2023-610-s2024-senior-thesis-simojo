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

### The Need for Generalized Autonomous Navigation

In addition to object detection and spatial awareness, autonomous navigation is
difficult to develop for general applications.

<!-- FIXME: continue to develop idea about why autonomous navigation is not
perfect. -->

Autonomous navigation is necessary for a robotic system to interact with its
surroundings in a real world environment, and it is necessary to realize
technologies such as fully autonomouos UAVs and vehicles. Modern robotic systems
employ a variety of techniques to achieve spatial awareness. These systems take
the form of ranging sensors (acoustic or optical) or optical flow, which is a
steady stream of camera information that is used to make assertions about the
relative positions of objects. Interpreting reliable and fast 3D spatial data
via optical flow requires extensive training of a convolutional neural network
and large amounts of data. Although recent work has enabled a racing quadcopter
to outperform professional pilots using optical flow, boasting a speed of $22
\frac{\text{m}}{\text{s}}$,

<!-- FIXME: try to give a proper motivation for why my approach has validity -->
<!-- FIXME: set it up for my specific project -->

## Goals of the Project

This project aims to train the COEX Clover quadcopter to perform basic
navigation and obstacle avoidance in randomized scenarios. By training the
quadcopter to explore an environment 

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
+@tbl:uavincidents. These cases provide a mere glimpse of the potential misuse
for UAV technology.

Table: Examples of Incidents that Highlight the Safety, Ethical, and Privacy
Concerns Related to UAV Operation (Source: @cummings2017). {#tbl:uavincidents}

Incident(s) | Significance | Source
:---|:---|:---:
UAV crashes in an Australian triathlon injuring an athlete. | UAV operator claimed someone hacked the device resulting in the crash. | [@martin2014]
Drone carrying Albanian flag sparked brawl between Serbian and Albanian players. | Incident viewed as “political provocation” by Serbian Foreign Minister, reopening old tensions. | [@thetelegraph2014]
FAA reports an increasing number of UAVs being sighted by commercial airlines | UAVs encroaching on commercial airspace, increasing safety concerns for commercial airlines and passengers. | [@jansen2015]
UAV on the White House Lawn | Breach of national security | [@berman2015]
UAV carrying radioactive materials lands on Japanese Prime Minister’s Office | UAVs used to make a political point in response to Japan’s damaged nuclear reactor | [@anderson2015]

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

<!-- FIXME: do I need any more sources or a better closing sentence here? -->

### The Ethics of This Project

`FIXME: I do not know how this project may mitigate the possible ethical
concerns.`

# Related work

#### GPS-Denied Positioning Using ArUco Markers

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

While this this project may not use ArUco markers, a comparison can be made
between the effectiveness of ArUco markers and an array of ToF sensors. For
determining the local position of the quadcopter.

#### FIXME

Path planning FIXME [@gugan2023].

# Method of approach

In this project, the COEX Clover quadcopter is used.

## Theory

With the earth's reference frame as $R^{E}$ and the quadcopter's body's
reference frame as $R^{b}$, the *attitude* of the quadcopter is known by the
orientation of $R^{b}$ with respect to $R^{E}$. We determine this from the
rotational matrix defined in +@eq:rotationalmatrix [@doukhi2022].

$$
\begin{bmatrix}
c \phi c \theta & s \phi s \theta c \psi - s \psi c \phi & c \phi s \theta c \psi + s \psi s \phi \\
s \phi c \theta & s \phi s \theta s \psi + c \psi c \theta & c \phi s \theta s \psi - s \phi c \psi \\
-s\theta & s \phi c \theta & c \phi c \theta
\end{bmatrix}
$$ {#eq:rotationalmatrix}

<!-- FIXME: what is the Newton-Euler formulation? -->

# Experiments

## Experimental Design

## Evaluation

## Threats to Validity

# Conclusion

## Summary of Results

## Future Work

## Future Ethical Implications and Recommendations

## Conclusions

# References

::: {#refs}
:::
