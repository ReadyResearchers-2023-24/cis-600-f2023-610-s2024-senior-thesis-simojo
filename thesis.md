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

The use of unmanned aerial vehicles (UAVs) is becoming increasingly ubiquitous
across various application domains, including real-time monitoring, wireless
coverage, remote sensing, search and rescue, and delivery of goods. UAVs are
thought of to be especially fit for search and rescue applications, given the
dangerous conditions associated with these operations. [@shakhatreh2019]

<!-- FIXME: need to provide tangible data right here -->
<!-- FIXME: provide data from the FAA here. -->

There are numerous hindrances that pose a barrier to ready, widespread adoption
of UAVs in the industry. UAV cameras are still very prone to over and under
exposure when navigating outdoors. This can interfere with the photogrammetric
processes used by many implementations. *Photogrammetry* refers to the
conversion of a set of 2D images to a 3D model. It can be used in place of
LiDAR or in conjunction with LiDAR. UAV camera systems can be delayed in their
response to higher and lower exposure of the course of a flight because of the
presence of sunlight and shaded areas found outdoors.

Using ranging sensors can mitigate many of the problems associated with UAVs
that rely on optical flow. Although ranging sensors come in varying forms, they
all rely on measuring the Time of Flight (ToF) of an emitted beam of light, that
is, the difference in time $\Delta t$ between $t_{\text{emitted}}$ and
$t_{\text{received}}$. In robotic applications, manufacturers have opted to use
laser diodes in the near infrared (NIR) band because of their inexpensiveness,
which is a result of the exploding fiber optics industry. Additionally, NIR
light is invisible and less harmful the human eye, giving it credibility in
terms of safety. [@raj2020, p. 16]

In addition to object detection and spatial awareness, autonomous navigation is
difficult to develop for general applications. 

<!-- FIXME: continue to develop idea about why autonomous navigation is not
perfect. -->

Robotic navigation is necessary for a robotic system to interact with its
surroundings in a real world environment, and it is necessary to realize
technologies such as fully autonomous vehicles and fully autonomous UAVs. Modern
robotic systems employ a variety of techniques to achieve spatial awareness.
These systems take the form of ranging sensors (acoustic or optical) or optical
flow, which is a steady stream of camera information that is used to make
assertions about the relative positions of objects. Interpreting reliable and
fast 3D spatial data via optical flow requires extensive training of a
convolutional neural network and large amounts of data. Although recent work has
enabled a racing quadcopter to outperform professional pilots using optical
flow, boasting a speed of $22 \frac{\text{m}}{\text{s}}$,

<!-- FIXME: try to give a proper motivation for why my approach has validity -->

## Goals of the Project

This project aims to train the COEX Clover quadcopter to perform basic
navigation and obstacle avoidance in randomized scenarios. 

## Ethical Implications

<!-- spying
war use
war use!!!
war use!!!!!!! -->

With the advent of autonomous UAVs, the FAA reports that over 1.47 million new
recreational drone owners registered in the United States between December 21,
2015 and the end of December 2022. In 2022, the FAA saw an average of 7,866
newly registered recreational UAV owners per month. [@faaForecast, p. 45] A
higher volume of recreational users implies the increased risk of misuse across
various domains.

Although the rate of increase of recreational UAV owners has been reported to be
slowing in recent years, the UAV industry is 

Any quadcopter equipped with one or more cameras can be considered a risk to
privacy. A flying system that can be remotely operated has the potential to be
exploited to infringe on privacy. Further, an autonomous system could add a
layer of anonymity to enable a malicious party to perform simultaneous
operations for the purpose of infringing on privacy. [@cummings2017]

@cummings2017 gathered five incidents in the past decade that have marked
ethical concerns related to UAV operation, which are listed in
+@tbl:uavincidents

Incident(s) | Significance | Source
:---|:---|:---:
UAV crashes in an Australian triathlon injuring an athlete. | UAV operator claimed someone hacked the device resulting in the crash. | [@martin2014]
Drone carrying Albanian flag sparked brawl between Serbian and Albanian players. | Incident viewed as “political provocation” by Serbian Foreign Minister, reopening old tensions. | [@thetelegraph2014]
FAA reports an increasing number of UAVs being sighted by commercial airlines | UAVs encroaching on commercial airspace, increasing safety concerns for commercial airlines and passengers. | [@jansen2015]
UAV on the White House Lawn | Breach of national security | [@berman2015]
UAV carrying radioactive materials lands on Japanese Prime Minister’s Office | UAVs used to make a political point in response to Japan’s damaged nuclear reactor | [@anderson2015]

Table: Examples of Incidents that Highlight the Safety, Ethical, and Privacy
Concerns Related to UAV Operation (Source: @cummings2017). {#tbl:uavincidents}


<!-- FIXME: data on infringements of privacy -->

With the advent of autonomous UAVs, [@faaForecast, @chavez2023]

<!-- In addition, reflect on ways that the above harms can be or are mitigated by your work -->

# Related work

In an effort to demonstrate the ability of a quadcopter to perform basic
navigation, @bogatov2021 used a grid of ArUco markers to provide the COEX CLover
4 quadcopter with an optical point of reference. @bogatov2021 states: "An ArUco
marker is a synthetic square marker composed by a wide black border and [an]
inner binary matrix which determines its identifier (id)." Using its on-board
camera, the drone resolved its own position by comparing it relative to each
ArUco marker. By default, the ROS module `aruco_detect` for the Clover 4 is
capable of publishing the positions of ArUco markers as *TF frames*, which is a
data type ROS uses to standardize multiple frames of reference in the context of
a global frame of reference. [@clover] With the goal of tracing out four
characters along an invisible plane using the drone's motion, the group
demonstrated that their method of using ArUco markers kept the quadcopter's
position within 0.1 to 0.2m of the desired waypoints, on average. They attribute
haphazard positioning of the ArUco markers and improperly defined PID controller
values to the cause of the error. [@bogatov2021]

Path planning FIXME [@gugan2023]

# Method of approach

## Theory

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
