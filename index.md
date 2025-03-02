---
layout: project_page
permalink: /

title: UltraTac &#58;Integrated Ultrasound-Augmented Visuotactile Sensor for Enhanced Robotic Perception
authors:
    Junhao Gong<sup>1*</sup>, Kit-Wa Sou<sup>1*</sup>, Shoujie Li<sup>1†</sup>, Changqing Guo<sup>1</sup>, Yan Huang<sup>1</sup>,  <br>   
    Chuqiao Lyu<sup>1</sup>, Ziwu Song<sup>1</sup>, Wenbo Ding<sup>1†</sup>  <br> 
affiliations:
    <small>*Indicates Equal Contribution; †Indicates Corresponding Authors.</small> <br> 
    1.Tsinghua University
paper: https://scholar.google.com/
hardware: https://scholar.google.com/ 
code: https://scholar.google.com/
video: https://scholar.google.com/
---

<!-- Using HTML to center the abstract -->
<div class="columns is-centered has-text-centered">
    <div class="column is-four-fifths">
        <h2>Abstract</h2>
        <div class="content has-text-justified">
        <!-- <span style="font-size:24px;"> -->
            Visuotactile sensors provide high-resolution tactile information but are incapable of perceiving the material features of objects. We present UltraTac, an integrated sensor that combines visuotactile imaging with ultrasound sensing through a coaxial optoacoustic architecture. The design shares structural components and achieves consistent sensing regions for both modalities. Additionally, we incorporate acoustic matching into the traditional visuotactile sensor structure, enabling the integration of the ultrasound sensing modality without compromising visuotactile performance. Through tactile feedback, we can dynamically adjust the operating state of the ultrasound module to achieve more flexible functional coordination. Systematic experiments demonstrate three key capabilities: proximity detection in the 3–8 cm range (R² = 0.99), material classification (Precision: 99.20%), and texture-material dual-mode object recognition achieves 92.11% accuracy on a 15-class task. Finally, we integrate the sensor into a robotic manipulation system to concurrently detect container surface patterns and internal content, which verifies its promising potential for advanced human-machine interaction and precise robotic manipulation.
        <!-- </span> -->
        </div>
    </div>
</div>

---


## Major Contributions
<!-- <span style="font-size:24px;"> -->
1.Integrated Sensor Architecture: Combines visuotactile imaging and ultrasound using a compact, coaxial design with a thin elastomer and optimized PZT transducer.<br>
2.Ultrasound-Enhanced Sensing: Enables non-contact proximity detection and contact-based material classification with dynamic mode switching.<br>
3.Experimental Validation: Demonstrates robust performance in object recognition tasks, supporting applications in robotics and inspection.<br>
<!-- </span> -->

## Structure Illustration
<!-- <div style="text-align: center;"> -->
<!-- <img src="/static/image/FigStructure.png" alt="FigStructure" style="width:85%;" /> -->
<!-- </div> -->
![Fabrication](/static/image/FigStructure.png)
<br>
UltraTac delivers dual-modal perception by integrating surface texture information from camera imaging through an elastomer membrane with material properties detected via ultrasound, as shown in the figure above. We optimize the sensor's mechanics, materials, electronic systems, and algorithms to achieve effective dual-modal sensing capabilities.
<br><br><br>




## Sensor Fabrication
<!-- <div style="text-align: center;">
<img src="/static/image/FigureFra.png" alt="FigureFra" style="width:100%;" />
</div> -->
![Fabrication](/static/image/FigureFra.png)
<br>
The fabrication process follows an eight-step procedure, as shown in the figure above, with parallel assembly for the upper and lower sensor components.

#### Upper Housing Assembly  
A 0.7 mm acrylic layer is bonded to the ring-shaped upper housing with cyanoacrylate adhesive, serving as both an acoustic matching and deformation platform. A 30:1 PDMS elastomer layer is cast, followed by a spin-coated (3000 rpm, 30s) HGM–PDMS membrane (1:1) for light blocking and ultrasound transmission. The assembly is heat-cured at 60 °C for three hours.

#### Lower Housing Assembly  
The camera module is centered in the lower housing, surrounded by an LED ring for uniform illumination. The annular PZT transducer is aligned coaxially with the optical axis, and tungsten-loaded epoxy (3:2) is injected behind it for acoustic backing. This assembly is also heat-cured at 60 °C for three hours. Finally, both parts are aligned and combined into the coaxial sensor module.

<br><br><br>




## System Workflow
<!-- <div style="text-align: center;">
<img src="/static/image/FigWorkflow.png" alt="Fabrication" style="width:50%;" />
</div> -->
![Fabrication](/static/image/FigWorkflow.png)
<br>
The pipeline is structured into three hierarchical levels: sensor, preprocessing, and processing.

#### Sensor Level  
The ultrasound module consists of a transducer connected to a sensor AFE, which extracts echo envelopes and communicates bidirectionally with the MCU for gain control. Meanwhile, the camera module captures tactile images in parallel.

#### Preprocessing Stage  
Ultrasound signals undergo Kalman filtering for noise reduction, while tactile images are augmented. A synchronization mechanism aligns the 50 Hz ultrasound data with the 30 Hz visual data by pairing each ultrasonic measurement with its nearest camera frame.

#### Processing Level  
Upon detecting touch, a dual-pathway approach is triggered. A camera-based touch signal determines whether ultrasound data is routed to Time-of-Flight (ToF) detection for distance estimation in non-contact scenarios or to XGBoost for material classification using Fourier features in contact scenarios. Simultaneously, a ResNet model processes tactile images for texture recognition\cite{resnet}.
<br><br><br>



## Timeline of System
<!-- <div style="text-align: center;">
<img src="/static/image/FigApplication3.png" alt="FigApplication3" style="width:100%;" />
</div> -->
![Fabrication](/static/image/FigApplication3.png)
<br>
The inspection process follows a structured timeline and consists of three phases: approach and grasp, touch, and take.

#### Approach and Grasp (0–2s)  
The ultrasound modality enables proximity sensing using Time-of-Flight (ToF) calculations to determine the optimal grasp position accurately.

#### Touch Phase (2s–onward)  
Uphe tacton contact, the sensor transitions its working mode. Tile modality captures surface textures, such as a hexagonal pattern, while the ultrasound modality switches to material detection, analyzing internal contents through spectral echo characteristics.

#### Take Phase  
The robotic arm transports the container to its designated location, guided by both surface patterns and internal content analysis.

<br><br><br>



## Video
<br><br><br><br><br>

## Citation


<!-- 
@article{turing1936computable,
  title={On computable numbers, with an application to the Entscheidungsproblem},
  author={Turing, Alan Mathison},
  journal={Journal of Mathematics},
  volume={58},
  number={345-363},
  pages={5},
  year={1936}
} -->

<!-- ## Background
The paper "On Computable Numbers, with an Application to the Entscheidungsproblem" was published by Alan Turing in 1936. In this groundbreaking paper, Turing introduced the concept of a universal computing machine, now known as the Turing machine.

## Objective
Turing's main objective in this paper was to investigate the notion of computability and its relation to the Entscheidungsproblem (the decision problem), which is concerned with determining whether a given mathematical statement is provable or not.


## Key Ideas
1. Turing first presented the concept of a "computable number," which refers to a number that can be computed by an algorithm or a definite step-by-step process.
2. He introduced the notion of a Turing machine, an abstract computational device consisting of an infinite tape divided into cells and a read-write head. The machine can read and write symbols on the tape, move the head left or right, and transition between states based on a set of rules.
3. Turing demonstrated that the set of computable numbers is enumerable, meaning it can be listed in a systematic way, even though it is not necessarily countable.
4. He proved the existence of non-computable numbers, which cannot be computed by any Turing machine.
5. Turing showed that the Entscheidungsproblem is undecidable, meaning there is no algorithm that can determine, for any given mathematical statement, whether it is provable or not.

![Turing Machine](/static/image/Turing_machine.png)

*Figure 1: A representation of a Turing Machine. Source: [Wiki](https://en.wikipedia.org/wiki/Turing_machine).*

## Table: Comparison of Computable and Non-Computable Numbers

| Computable Numbers | Non-Computable Numbers |
|-------------------|-----------------------|
| Rational numbers, e.g., 1/2, 3/4 | Transcendental numbers, e.g., π, e |
| Algebraic numbers, e.g., √2, ∛3 | Non-algebraic numbers, e.g., √2 + √3 |
| Numbers with finite decimal representations | Numbers with infinite, non-repeating decimal representations |

He used the concept of a universal Turing machine to prove that the set of computable functions is recursively enumerable, meaning it can be listed by an algorithm.

## Significance
Turing's paper laid the foundation for the theory of computation and had a profound impact on the development of computer science. The Turing machine became a fundamental concept in theoretical computer science, serving as a theoretical model for studying the limits and capabilities of computation. Turing's work also influenced the development of programming languages, algorithms, and the design of modern computers.

## Citation
```
@article{turing1936computable,
  title={On computable numbers, with an application to the Entscheidungsproblem},
  author={Turing, Alan Mathison},
  journal={Journal of Mathematics},
  volume={58},
  number={345-363},
  pages={5},
  year={1936}
}
``` -->
