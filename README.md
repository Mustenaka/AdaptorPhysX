# AdaptorPhysX

This project is not finished, i will push it on next spring.

AdaptorPhysX full name is: Adaptive Physics Extension, abbreviation: APEX. this project try to use PBD (position based dynamics) algorithm to simulate a deformable object with volume or surface, and implement a calculation framework according to PBD calculation rules.
PBD algorithm is more used in non-rigid body fabric algorithm, so I decided to try this algorithm applied to the settlement of flexible body.

What is PBD: https://matthias-research.github.io/pages/publications/posBasedDyn.pdf

## Require

 Require unity ECS architecture. (Entities is not necessary), plz Windows – Package Manager – Add package from git URL - input: com.unity.jobs 

- com.unity.burst - 1.8.9
- com.unity.jobs  - 0.70.0-preview.7
- com.unity.mathematics - 1.2.6

## Models

This project contains the following modules:

![AdaptorPhysX-moduleDiagram](./Pic/AdaptorPhysX-moduleDiagram.png)

## Effect

##### Rope:

1 rope, with 100 particle, iterator 10, constraint have: distance, no optimize

![rope-100p-constraint-distance](./Pic/Effect/Rope/rope-100p-constraint-distance.gif)

(waiting for upload)



## Custom, rewrite

(waiting for project finished)



About：

My blog about PBD，URL：https://www.mustenaka.cn/index.php/2023/09/06/pbd-method-learn-01/

Adaptive Physics Extension: AdaptorPhysX, APEX

