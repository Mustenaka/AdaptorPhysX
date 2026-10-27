# 待办清单

本文件列出与 Obi 生态、医疗仿真需求相比尚缺的功能，以及基准测试与代码检查中发现的缺陷，并逐项对照当前实现。
每个条目有固定编号，新对话中直接引用编号（例如“处理 BUG-2”）。

- 对照基准：提交 `17fde10`（2026-10-26），分支 `dev/softbody`。文件行号以 `20c1e5e` 为准。
- 里程碑编号（M1–M5）见 `docs/ROADMAP.md`；算法与数据结构见 `docs/DESIGN.md`。

## 状态定义

| 状态 | 含义 |
|---|---|
| 已完成 | 已实现并有测试或基准数据 |
| 部分完成 | 已实现一部分，未完成的部分在“现状”中列出 |
| 未开始 | 没有实现 |
| 原因未查明 | 缺陷，有复现数据，原因未查明 |
| 原因已确认 | 缺陷，原因已确认，未修复 |

## 新对话的工作方式

1. 读 `README.md`、本文件中对应条目、`docs/DESIGN.md` 中条目引用的章节。
2. 按条目的“验收”完成；按 `docs/ROADMAP.md` 的工作约定附 EditMode 测试，影响效果的改动附标准场景截图。
3. 涉及性能或切割行为的改动，运行基准测试并与 `docs/DESIGN.md` 6.1 节对比：
   - 网格：`sh Benchmarks/fetch-meshes.sh`（兔子、龙；box 与肝形网格由代码生成）。
   - 编辑器未打开工程时用 batchmode（命令见 README）。
   - 编辑器已打开工程时，通过 MCP 调用 `ApxBenchmark.ScheduleKeepingScenes(settings)`。
     不要同步调用 `RunKeepingScenes`：完整运行需要约 150 s，超过 MCP 应答超时后命令会被重发，基准测试会重复运行。
   - 同一机器、同一构建下各阶段的 `positionsHash` 应一致，可用于判断改动是否改变了仿真结果。
4. 完成后更新本文件中条目的状态与现状，并在“变更记录”中加一行。

## 1. 缺陷

| 编号 | 问题 | 现状与证据 | 状态 | 下一步 | 验收 |
|---|---|---|---|---|---|
| BUG-1 | 放在碰撞体上的物体全切后不分离 | DESIGN 6.1：box 64、bunny 32/48/64、dragon 32 的全切断开了棱，块数仍为 1（不计节点数低于 2% 的碎块）。box 64 在无重力、无碰撞体时同一刀路分成两块。有重力时，刀到达之前已有 1259 个节点越过刀面（刀面距最近节点 0.5h）。把 BUG-6 的规则扩展到刀刃前方（交点在刀刃前方、距扫掠四边形不超过静止 SDF 时也断开）后，bunny 32、48 分成两大块（5720 / 5169、17995 / 13799），但单节点碎块增多（bunny 64 由 23 块增至 33 块），说明不分离与刀刃前方表面外的棱有关。用 BUG-2 的实验求解器（分辨率 64 的漂移由 1.75 mm 降到约 0.2 mm）运行基准测试，box 64 全切分成两块（42504 / 40656），liver 各分辨率不再有碎块，说明 box 64 的不分离由漂移引起；但 bunny 16 变为不分离。原因：刀沿 x 推进约 60 帧，期间组织相对刀面移动（落地漂移、切开后的回弹、被刀带动的部分），刀经过一条棱两端的时刻不同。bunny 32 中有 22 条未断开的棱，两端在刀经过时分处刀面两侧，但每次被测试时都不跨刀面（dragon 32 有 12 条，box 32 为 0）。修复：同一刀路（`ApxScalpel` 的一次连续切割）内，节点记住刀经过时所在的侧别（`StrokeSide`，刀面两侧 3h 内、投影落在扫掠四边形上的节点）；两端都有侧别的棱按侧别是否不同决定断开；夹在两种侧别之间的无侧别节点按当前位置补记侧别；侧别在刀推进 8h 后失效，刀路可以再次经过同一处组织。结果（2026-10-27）：16 组全切均分成两大块，bunny 64 为 46687 / 22856 另有 9 个不超过 2 个节点的碎块，dragon 32 另有 13 个节点、dragon 48 另有 38 与 9 个节点的块（均低于 2%）；单节点碎块大多消失；静止、落下两阶段的 `positionsHash` 不变，切割帧扫掠耗时不变 | 已完成 | — | `FullCutOfABodyLyingOnAColliderSeparates`：box 分辨率 64 落到碰撞盒（摩擦 0.6）上后全切，得到 2 块（不使用刀路侧别时为 1 块） |
| BUG-2 | 摩擦使竖直落下的物体整体侧向漂移 | box 从 5 mm 高处落到 Box 碰撞体上，180 步后节点平均 z 位移：摩擦 0.6 时 +1.35 mm（分辨率 32）、+1.75 mm（分辨率 64，x 方向 +0.6 mm）；摩擦 0 时约为 0。物体绕竖直轴转 90°、180° 后，漂移方向随 lattice 转动，大小不变（1.353 mm），说明来源是离散与求解顺序：每个子步一次 Gauss-Seidel，切片内按 +z 顺序、体素内 6 个四面体按 x、y、z 面的顺序求解，且所有体素用同一个 Kuhn 剖分；落地时底层节点在每个子步有同向的偏移，摩擦把它保留下来。实验（分辨率 32 / 64，z 位移）：每个子步或每步交替正反顺序，物体不停振荡，分辨率 64 发散；切片内按 (y, z) 奇偶分 4 类依次求解，1.21 / 2.11 mm；再按体素坐标奇偶镜像剖分（相邻体素共享面上的对角线一致），0.68 / 0.33 mm（与 lattice 对齐的对称 box 也有 0.32 mm）；再在奇数切片中反转 4 类的顺序，0.63 / 0.19 mm（含 x 方向）。最后一种在基准测试中改善了 box 64 与 liver 的全切（见 BUG-1），但 bunny 16 变为不分离，bunny 48、dragon 64 碎块增多，box 48 每步耗时 9.1 → 14.4 ms，落下后体积比略降，未采用。注意 DESIGN 第 3 节：早期的奇偶交替 5 四面体剖分在切口边缘产生过两个体素周期的锯齿。每个子步正向、反向各扫描一次（对称 Gauss-Seidel，λ 不累积）：漂移 1.95 / 1.44 mm（16 子步）、−0.38 / −1.31 mm（8 子步），刚度约变为两倍（自重下沉为解析值的 0.54 倍），未采用；说明漂移主要来自所有体素相同的剖分，而不是扫描顺序。BUG-1 修复后漂移不再使全切失败，影响限于约 1–2 mm 的位移 | 原因已确认 | 候选：每个体素同时使用两种对称剖分（四面体数加倍）；接触层节点用 Jacobi 方式求解；或修改摩擦，使其不保留求解器在一个子步内给接触面的同向偏移 | 摩擦 0.6 时平均水平位移小于 0.05 mm；新增 EditMode 测试 |
| BUG-7 | 绕竖直轴旋转放置的物体在无外力时变形 | 已修复。四面体旋转的四元数初始为单位四元数，每个子步的更新量限制在 0.5 rad 以内；物体转 90° 时要经过数个子步才找到旋转，期间弹性约束使物体形变（无重力 30 帧内最大位移 159 mm），转 180° 时更新量为 0，始终找不到。修复：创建时四元数取静止空间到世界空间的旋转（`SoftBodySimulation.RebuildTopology`） | 已完成 | — | `RotatedBodyStaysAtRestWithoutGravity`（转 90°、180°，30 帧最大位移小于 0.1 mm；修复前 159 mm）。未旋转物体的基准测试 `positionsHash` 不变 |
| BUG-3 | 落下后的体积损失随分辨率增大 | 6.1 节落下 1.5 s 后表面体积比：box 0.996（16）→ 0.972（64），bunny 64 为 0.946。ν = 0.45。原因是求解器收敛不足：每个子步一次 Gauss-Seidel、λ 每子步从 0 开始，恒定载荷下的稳态误差约为每个体素 g·Δt²，沿载荷路径累加，体素越多越大。box 落到碰撞盒上的 lattice 体积比（分辨率 16 / 32 / 64）：8 子步 0.986 / 0.965 / 0.884，16 子步 0.995 / 0.990 / 0.972，32 子步 0.998 / 0.997 / 0.991；顶面下沉在 32 子步时趋于约 0.5 mm（物理值），16 子步时为 0.63 / 1.02 / 2.24 mm，6 s 后不变。在预测后、求解前也做一次碰撞投影，结果不变（排除碰撞顺序） | 原因已确认 | 求解器层面：子步间沿用 λ（warm start）或 PERF-9 中的加速方法；或子步数随分辨率增加（分辨率 64 时耗时约加倍）。与 PERF-9 一起评估 | 分辨率 16–64 之间体积比差异小于 1% |
| BUG-4 | 刀面经过一层 lattice 节点时不分离 | 已修复。原因：`SegmentTriangle` 对端点在刀面上的棱接受 t = 0 与 t = 1，节点层两侧的棱都被断开，节点层成为单独一块（box 分辨率 16 修复前为 2916 / 2592 / 324 三块）。修复：`CutJob.Crossing`（`TopologyJobs.cs`）先按端点到扫掠三角形平面的有符号距离分侧，距离为 0 归正侧，同侧不断开；`SegmentTriangle` 在端点处因舍入漏判时，直接检验交点是否在三角形内 | 已完成 | — | `FullCutThroughALayerOfNodesSeparatesTwoPieces`：box 分辨率 16 沿一层节点全切，得到 2 块。基准测试 16 组的块数、断棱数与 4 个阶段的 `positionsHash` 与修复前一致 |
| BUG-5 | 参数的生效时机不一致 | 已修复。此前 `preStrain` 与三个 `skin*` 参数只在构造时使用，运行时修改无效；`density` 只在拓扑重建中使用，要到下一次切割才生效。修复：`SoftBodySimulation.ScheduleStep` 发现预应变或皮肤参数变化时重算各体素的静止形状与刚度（`UpdateCellRest`）；有效逆质量乘以“建立质量时的密度 / 当前密度”，下一次拓扑重建时按新密度重建质量；`ApxSoftBody.Rebuild()` 按当前设置重新体素化，钉住区域重新从 `pinColliders` 取得；运行时修改 `resolution` 时输出警告，提示调用 `Rebuild()`。参数不变时基准测试 `positionsHash` 不变 | 已完成 | — | `DensityChangeAppliesOnTheNextStep`（密度加倍后自重下沉为 2 倍）、`PreStrainChangeAppliesOnTheNextStep`（运行时设 0.1，自由物体缩到 0.9）、`RebuildAppliesResolutionAndKeepsPins` |
| BUG-6 | 刀刃从表面穿出、但未到达表面外的 lattice 节点时不分离 | 已修复。原因：边界体素的节点最多位于表面外约一个体素，只有这些节点之间的棱也断开，两侧才分离。复现：演示中的皮瓣（厚 16 mm，分辨率 36，预应变 0.2），15 mm 刀刃保持起刀高度，从中部切向近侧边缘，近侧边缘附近有 2–4 条底层棱未断开，边缘连在一起。修复：两端 SDF > 0 的棱，交点在刀尖或刀跟外侧、在本步新扫过的范围内，且扫掠四边形到交点的距离不大于该处静止 SDF 时断开（`CutJob.OutsideCrossing`，DESIGN 4.1 节）。启用该规则后，皮瓣切口上出现拉长的皮肤条。原因在 `SurfaceMesh.ResolveBySides`：皮肤块的顶点取块所在一侧的最近体素角点的副本，角点的侧别按到拟合平面的距离判断；拟合平面相对断棱倾斜，距平面很近的角点被判到另一侧（体素 (14, 2, 0) 中 −x 副本的角点在平面正侧 4 µm 处），顶点用切口另一侧的副本求值。不启用规则、刀面靠近一层节点时也会发生。另一种情况：相邻体素对同一切口拟合的平面在三角形处相距约 0.09h，`CollectPlanes` 按原点处的偏移量去重，两个平面都保留，两平面之间的窄块的顶点在两个体素中分属两侧。修复：角点侧别按其副本的边界断棱判断（`SurfaceMesh.CornerSide`）；平面按三角形顶点处的距离去重（DESIGN 第 5 节） | 已完成 | — | `BladeLeavingThroughTheSurfaceSeparatesBeforeReachingTheOuterNodes`：box 分辨率 8，刀尖在底面以下 0.2h 时全切得到 2 块，在底面以上 0.2h 时为 1 块。`SkinFlapTests.CutOutThroughTheEdgeLeavesNoSkinAcrossTheIncision`：皮瓣，15 mm 刀刃跟随表面，从 z = 0.02 m 切到 −0.09 m，x = −0.02 m（60 帧）与 x = −0.012 m（40 帧），没有长于 2h 的皮肤三角形（修复前 59、6 个）。同一刀路取 x = −0.02、−0.012、−0.0105、−0.0075 m，10–120 帧，规则启用与不启用共 56 组，均没有长于 2h 的皮肤三角形（修复前 10 组有，最多 62 个）。基准测试 16 组的块数、断棱数与 4 个阶段的 `positionsHash` 与不启用规则时一致 |
| BUG-8 | 持续挤压时软体之间相互穿透 | 原因（2026-10-27 确认）：(1) 对方副本每步复制一次、按份额推出（`PartnerShare`）、每子步推出量上限 0.25h，使持续载荷下本物体弹性推入的部分不能完全推出，残余重叠随载荷增大；(2) 节点越过对方 SDF 的精确带（1.5 个体素，更深处取 −2h）后梯度小于 0.5，不再被检测；(3) 对方 SDF 中轴处梯度为 0，同样不被检测。已修改（DESIGN 9.4 节）：相互接触的物体同步推进子步（`SubstepGroup`），每个子步的约束求解之后各物体把副本刷新为对方求解后的位置，两个物体看到同一重叠、各推出自己的份额；本步不推进的对方作为静止障碍；接触另用精确带以外补全的格点 SDF（`ContactSdfJob`）。结果：静态挤压（`SqueezedBodiesKeepTheirContactUnderASustainedLoad`）最深节点 4.43 mm，停住期间不增加，节点速度 0.001 m/s（修改前由 4.5 mm 增至 6.2 mm，节点速度 1.4 m/s）；无摩擦地面上的推动（`PinnedBodyPushesAnotherAlongAFrictionlessFloor`）最深节点 4.33 mm，被推物体前移 75.5 mm；`StackedBodiesRestOnEachOther` 静止后最大节点速度 0.007 / 0.003 m/s（修改前 0.020 / 0.013），下方块休眠。未解决：ECO-1 验收场景（地面摩擦 0.6）中被推物体向前倾倒（推动第 15 帧顶部前移 16.9 mm、底部 9.0 mm，背面底部抬起约 4 mm），接触集中在推动体底部两行节点，这里对方没有节点进入推动体，推动体只推出自己的份额，残余重叠在推动第 9 帧为 1.6–2.1 mm 并继续增大，被推物体前移 5 mm 后停下，推动体穿过（最深 27.6 mm）。试过但未采用：中轴处改用副本角点的最小二乘梯度（堆叠与静态挤压中节点速度 0.6–0.9 m/s）；推出量上限放宽到 0.5h（该场景最深 8.8 mm、被推物体前移 47 mm，但静态挤压中节点速度 1.5 m/s）；由本物体推出自身超出预测部分、不分份额（动量不守恒，堆叠时节点速度 0.16–0.65 m/s）。 | 部分完成 | 处理受力集中的单侧接触，候选：(a) 每个子步做多次"刷新—接触"；(b) 节点推出时把反向修正按重心坐标与逆质量分给对方四面体的角点（由对方在下一个子步施加），使单侧接触也守恒动量；(c) 推出量上限按本子步自身的位移放宽。每项都要检查静态挤压与堆叠中的节点速度 | `PinnedBodyPushesAnotherWithoutPassingThrough` 去掉 `[Ignore]` 后通过；`SqueezedBodiesKeepTheirContactUnderASustainedLoad`、`PinnedBodyPushesAnotherAlongAFrictionlessFloor`、`StackedBodiesRestOnEachOther` 仍通过；S8 中接触的两个物体没有抖动；单物体基准测试 16 组的 `positionsHash` 不变 |

## 2. 与 Obi 生态的差距

| 编号 | 能力 | Obi | AdaptorPhysX 现状 | 状态 | 验收 |
|---|---|---|---|---|---|
| ECO-1 | 不同物体之间的碰撞 | 同一 solver 内全部物体互相碰撞 | 已实现（DESIGN 9.4 节）：每轮步进开始时复制对方附近的体素副本，节点按对方静止 SDF 在形变四面体上的插值推出，按质量分担，`collideWithBodies` 与 `collisionLayers` 过滤；相互接触的物体同步推进子步，每个子步刷新副本（BUG-8）。`StackedBodiesRestOnEachOther`：小块停在大块上（中心 57.6 mm，关闭接触时 18.4 mm）；S8 中肝叶停在组织块上。`SqueezedBodiesKeepTheirContactUnderASustainedLoad`、`PinnedBodyPushesAnotherAlongAFrictionlessFloor` 通过。限制：切面不在对方 SDF 中；相互接触的两个物体通常只有一个休眠；地面摩擦下的相向挤压测试未通过（BUG-8） | 部分完成 | 已通过：静止堆叠（`StackedBodiesRestOnEachOther`，中心 57.6 mm；关闭接触时 18.4 mm）。未通过：相向挤压（`PinnedBodyPushesAnotherWithoutPassingThrough`，标为 `[Ignore]`，见 BUG-8）：一个物体由钉住区域（`pinColliders`）以 0.1 m/s 推向另一个放在地面上的物体，挤压期间任一节点穿入对方表面的深度不超过 1h（深度按物体间接触的定义：对方静止 SDF 在形变四面体上插值，除以梯度长度），被推物体重心沿挤压方向位移大于 1h；测试同时检查关闭物体间接触时推动体进入对方超过 1h、被推物体位移小于 1h。属于 M3 |
| ECO-2 | 碰撞体类型 | 支持 MeshCollider 与距离场 | 已实现（DESIGN 9.1 节）：MeshCollider 运行时按网格烘焙距离场；`ApxDistanceField` 资产（编辑器中烘焙）经 `ApxColliderMaterial` 用于任意 Collider。测试：`DistanceFieldMatchesTheBoxDistance`（距离场本身）、`DistanceFieldFloorHoldsTheBodyLikeABoxCollider`（直接构造的距离场地面）、`MeshColliderAndFieldAssetFloorsHoldTheBodyLikeABoxCollider`（组件路径 `TryMakeFieldShape`：MeshCollider 运行时烘焙；`ApxDistanceField` 资产经序列化复制后使用，Collider 比资产网格高 50 mm，只用于范围查询）。S10 环形器械（MeshCollider 与 `Ring Field` 资产）压入组织，下降量 4.4 mm | 已完成 | `MeshColliderAndFieldAssetFloorsHoldTheBodyLikeABoxCollider`：10 cm 立方体（分辨率 8）落到 MeshCollider 地面、`ApxDistanceField` 资产地面上，最低节点高度与 Box 碰撞体之差均为 0.000 mm（要求小于 0.5 mm） |
| ECO-3 | 与刚体的双向耦合 | 有 | 已实现（DESIGN 9.3 节）：非运动学 Rigidbody 在步内由软体逐子步推进，步末把组织造成的位移、转动与速度修正加到 PhysX 的结果上（`twoWayCoupling`）。`RigidBodyRestsOnPinnedTissueAndBendsIt`：组织对钢球的力 0.976 N（重量 0.981 N），关闭耦合时钢球穿过组织；S9。限制：Rigidbody 的 Freeze 约束不参与 | 已完成 | 组织能推动带 Rigidbody 的器械 |
| ECO-4 | GPU 后端 | Obi 7 有 Compute Shader 后端 | 没有 | 未开始 | 属于 M4 |
| ECO-5 | 布料、绳、流体 | 有 | 已冻结（ROADMAP 决策 3）。医疗场景用途：膜与筋膜（布料）、缝线与血管（绳）、出血（流体） | 未开始 | 需先修改 ROADMAP 决策 3 |
| ECO-6 | 粘附与塑性 | 碰撞材质有 stickiness，softbody 有 plasticity | 已实现（DESIGN 9.5 节）：`ApxColliderMaterial.stickiness`、`stickDistance`（接触过的节点被拉回，超过粘附距离后脱离）；`plasticYield`、`plasticCreep`、`plasticRecovery`、`maxPlasticDeformation`（每个体素的塑性伸长，体积不变）。`StickyInstrumentPullsTheSurfaceAfterIt`（粘附时表面上移 1.00 mm，无粘附时 −0.05 mm）、`PressedPlasticBodyKeepsADent`（压痕 11.5 mm，弹性 0.12 mm）；S11 | 已完成 | 湿组织粘附器械；按压后保留残余形变 |
| ECO-7 | 编辑器工具 | blueprint 预计算；用笔刷绘制质量、钉住区域 | blueprint 已实现（DESIGN 9.7 节）：`ApxSoftBodyBlueprint` 资产保存细分网格与体素化结果，检视面板“Bake Blueprint”，匹配时 Start 不细分、不体素化，结果逐位一致（`BlueprintGivesTheSameSimulationWithoutVoxelizing`）。未实现：在编辑器中用笔刷绘制钉住区域与材料（钉住区域仍用 Collider 选取） | 部分完成 | 体素化结果存为资产，Start 不再体素化；在编辑器中绘制钉住区域与材料 |
| ECO-8 | 状态快照与撤销 | 有 | 已实现（DESIGN 9.6 节）：`SoftBodySimulation` / `ApxSoftBody` 的 `CaptureState`、`RestoreState`，含切割拓扑、塑性与抓取状态。`RestoredStateRepeatsTheSameSteps`：切割、恢复后再步进的节点位置与保存后直接步进逐位一致；`RestoredBodyRendersItsTopology` | 已完成 | 保存与恢复仿真状态（含切割拓扑），恢复后 `positionsHash` 与保存时一致 |

## 3. 医疗仿真需求

### 3.1 材料

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-1 | 应变硬化（J 形应力-应变曲线） | co-rotational 线弹性，小应变线性化（DESIGN 第 3 节） | 未开始 | 单轴拉伸曲线与所选本构模型的解析解一致 |
| MED-2 | 粘弹性（应力松弛、蠕变） | 没有 | 未开始 | 恒定应变下应力随时间衰减，衰减时间常数可配置 |
| MED-3 | 各向异性（肌纤维方向） | 没有 | 未开始 | 沿纤维方向与垂直方向的刚度比可配置并可测量 |
| MED-4 | 阻尼模型 | 全局速度阻尼（`SolverJobs.cs:157`），刚体运动也被衰减 | 未开始 | 自由下落与匀速平移不受阻尼影响 |
| MED-5 | 多材料与体素材料标签 | 两层：皮肤层与主体（DESIGN 第 3 节）。M5 列出脂肪、肌肉不同材料 | 部分完成 | 每个体素一个材料 ID；从 CT/MRI 分割标签（NIfTI）读取材料 ID |
| MED-6 | 超阈值撕裂 | 没有（M5） | 未开始 | 应变超过阈值时断棱，撕裂沿受力方向扩展 |

### 3.2 切割精度与器械

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-7 | 切口精度 | 分辨率上限 64（`ApxSoftBody.cs:22` 的 Inspector 范围），0.15 m 的物体约 2.3 mm 一个体素；薄于一个体素的结构无法分离（DESIGN 4.3 节）；刀路在体素内弯曲时切口为折线（DESIGN 第 7 节） | 未开始 | 切口附近局部加密 lattice；弯曲刀路的切口误差小于设定值 |
| MED-8 | 切割阻力与切割力 | 扫掠面与棱相交即断开，组织不先压陷；没有切割力输出 | 未开始 | 切入前组织压陷，切入条件取决于刀锋参数与组织韧性；输出刀刃受力 |
| MED-9 | 器械种类 | 只有线段刀刃（`ApxScalpel`）。缺：剪刀（两刃闭合）、电刀与超声刀（删除体素）、穿刺针（插入不切开）、夹闭器与吻合器。缝合见 3.3 节 | 未开始 | 每种器械一个标准场景截图与 EditMode 测试 |
| MED-10 | 镊子夹持 | 半径内节点设为运动学节点（`BeginGrab`），等同固定连接 | 未开始 | 两钳口夹持，接触带摩擦，拉力超过阈值时滑脱 |

### 3.3 缝合

缝合由三部分组成：缝针穿刺组织、缝线在穿刺通道内滑动、收紧缝线使切口两侧对合。SUT-1 是其他条目的基础；
SUT-5 不模拟缝线，可以在缝针与缝线之前实现，用于验证切口闭合与组织张力。

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| SUT-1 | 材料点锚定 | 抓取与钉住通过节点列表绑定到运动学帧（`Attach`，`SoftBodySimulation.cs:633`；最多 32 帧，`MaxKinematicFrames`，`SoftBodySimulation.cs:71`）；没有嵌入在体素副本内的材料点 | 未开始 | 材料点由（体素副本，局部坐标）定义，位置按三线性插值计算；切割后随副本继承；可作为穿刺点、缝线锚点、力反馈接触点使用 |
| SUT-2 | 缝针穿刺 | 没有（直针穿刺见 MED-9） | 未开始 | 弧形缝针沿轨迹刺入并穿出组织，不产生切口；进针力超过阈值才刺入；针与组织之间有摩擦；记录进针点与出针点（SUT-1 材料点） |
| SUT-3 | 缝线 | 没有。绳的仿真已冻结（ECO-5，ROADMAP 决策 3） | 未开始 | 缝线可弯曲、长度近似不变，与组织、器械和自身碰撞。开始前需修改 ROADMAP 决策 3 |
| SUT-4 | 缝线穿过组织 | 没有 | 未开始 | 缝线在穿刺通道内滑动，带摩擦；收紧缝线时切口两侧对合 |
| SUT-5 | 简化缝合（不模拟缝线） | 没有。另外，按 DESIGN 3.1 节，半切切口两侧属于同一块且静止距离小，不产生接触，对合后切面可能相互穿透 | 未开始 | 每一针为连接切口两侧两个材料点的距离约束，目标长度逐步减小到 0；S6 的切口在皮肤张力下被拉拢闭合；对合后两侧切面不相互穿透 |
| SUT-6 | 打结 | 没有 | 未开始 | 判定结的形成；成结后缝线两段之间改为固定约束，松开器械后缝线张力保持 |
| SUT-7 | 缝合后的拓扑 | 切割只减少连通（DESIGN 4.2 节），没有重新连接断棱的机制 | 未开始 | 单次训练的时长内一般不需要愈合；如需要，定义断棱重新连接与体素副本合并的规则，并保持 `positionsHash` 可复现 |
| SUT-8 | 缝合评分指标 | 没有 | 未开始 | 针距、边距（进针点到切口边缘的距离）、进针深度、对合程度、缝线张力；与 MED-13 使用同一事件接口 |
| SUT-9 | 标准场景 S7 | 没有 | 未开始 | 在 S6 的小切口上缝 3 针并收紧，切口闭合；加入 ROADMAP 标准场景表，截图工具输出 S7 |

### 3.4 力反馈（计算接口）

范围：只提供力的计算接口与线程间交换接口，不包含设备驱动（OpenHaptics、Force Dimension SDK 等）。
没有实际设备，所有条目的验收都以 EditMode 测试或脚本设备完成。按编号顺序实现，每个条目可单独在一次对话中完成。

约定：
- 单位为 N、N·m、m、s，坐标为世界坐标。力的方向为组织作用在器械上的力。
- 一个物理步内各子步结果的平均值为该步的力；可选输出逐子步序列。16 子步、60 Hz 时子步频率为 960 Hz，
  但这些值在一帧内一次计算完成，对设备有一帧的延迟。
- 求和不使用浮点原子操作：按批次求部分和，再按固定顺序归约，保持同一机器、同一构建下逐位可复现。

| 编号 | 内容 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| HAP-1 | 碰撞体受力：器械（Collider）受到的力、力矩、接触中心 | 已实现（DESIGN 9.2 节）：节点修正的冲量按 256 个节点一块累加、步末按固定顺序求和；`ApxSoftBody.TryGetColliderWrench`、`ColliderWrenches`、`ApxHaptics.TrySumColliderWrench`、`SoftBodySimulation.TryGetShapeWrench`。力矩参考点为碰撞体形状的参考点（球、盒、距离场为中心，胶囊为第一个端点），与下方草案中的 Transform 位置不同。`ColliderForceEqualsTheWeightOfARestingBody`：−9.790 N，重量 9.810 N（偏差 0.2%） | 已完成 | 节点 i 对碰撞体 s 的位置修正 Δx（法向投影加摩擦）换算为力 `m_i Δx / h²`（h 为子步长），对节点求和取反；力矩相对碰撞体 Transform 位置。静止物体放在 Box 碰撞体上，碰撞体受力等于总重量（误差 < 2%） |
| HAP-2 | 运动学帧受力：抓取帧、钉住区域的支座反力 | 没有。运动学节点在求解中逆质量为 0（`SolverJobs.cs:281`），作用在它们上的约束力不计算 | 未开始 | 两种候选方法：在 `SolveCellsJob` 中累加约束对运动学节点的 `λ∇C / h²`；或在步末对运动学节点计算弹性力 `−∂E/∂x`。按下面两项测试选择：用抓取提起整个物体静止后，抓取受力等于总重量（误差 < 2%）；两端钉住的物体静止时，钉住反力之和等于总重量 |
| HAP-3 | 力的物理校验测试 | 没有 | 未开始 | EditMode 测试：自重（HAP-1、HAP-2 中的两项）；力平衡（器械力 + 钉住反力 + 重力 = 0，物体静止时）；球形压头压入组织块，力随深度单调增加，小深度下与 Hertz 接触解比较（容差 30%，与 `SelfWeightCompressionMatchesYoungsModulus` 同级）；相同输入时力逐位一致 |
| HAP-4 | 局部线性力模型：供 1 kHz 线程在两次物理步之间计算力 | 没有 | 未开始 | 每步发布 `F(x) = F0 + K (x − x0)`，K 为 3×3 刚度（N/m）。K 的计算方法待定，候选：各接触节点法向与等效刚度的和 `Σ k n nᵀ`；相邻两步的有限差分（秩一更新）。验收：压头匀速压入时，用模型预测下一步的力，相对误差 < 10% |
| HAP-5 | 虚拟耦合（纯计算） | 没有 | 未开始 | `F = k_c (x_proxy − x_device) + b_c (v_proxy − v_device)`，带力上限；参数由调用方按设备给出。单元测试：脚本设备沿闭合轨迹往返时，设备对用户做的净功不为正（无能量注入） |
| HAP-6 | 线程间交换与设备抽象 | 没有 | 未开始 | 设备位姿由触觉线程写入、物理步读取；力模型由物理步发布、触觉线程读取；双缓冲，无锁，读写不阻塞主线程。`IApxHapticDevice` 预留给设备适配 |
| HAP-7 | 无设备测试 | 没有 | 未开始 | 脚本设备（`ApxScriptedHapticDevice`）在测试线程中以 1 kHz 回放录制或脚本轨迹并记录收到的力；EditMode 测试覆盖：物理帧率从 60 Hz 降到 20 Hz 时输出力连续（相邻 1 ms 的变化小于设定值） |

切割阻力与切割力输出见 MED-8，实现时复用 HAP-1 的累加方式。

接口草案（名称与签名在实现时可调整）：

```csharp
// 一个物理步的受力，组织作用在器械上
public struct ApxWrench
{
    public float3 Force;            // N
    public float3 Torque;           // N·m，相对 Origin
    public float3 Origin;           // 力矩参考点：碰撞体 Transform 位置，或运动学帧原点
    public float3 ContactCentroid;  // 按力大小加权的接触点
    public int ContactNodes;
}

public partial class ApxSoftBody
{
    // HAP-1：上一步中该碰撞体受到的力；本步未接触时返回 false
    public bool TryGetColliderWrench(Collider collider, out ApxWrench wrench);
    // HAP-2：抓取帧与钉住区域（pinColliders 的下标）受到的力
    public ApxWrench GrabWrench { get; }
    public ApxWrench GetPinWrench(int pinIndex);
}

public static class ApxHaptics
{
    // 所有 ApxSoftBody 对同一碰撞体的力之和
    public static ApxWrench SumColliderWrench(Collider collider);
}

// HAP-4：局部线性力模型，每个物理步发布一次
public struct ApxForceModel
{
    public double Time;        // 仿真时间，s
    public float3 Position0;   // 发布时的器械位置
    public float3 Force0;      // 发布时的力
    public float3x3 Stiffness; // dF/dx，N/m
    public float MaxForce;     // 力上限，N
    public float3 Evaluate(float3 position);
}

// HAP-5：虚拟耦合
public struct ApxVirtualCoupling
{
    public float Stiffness;    // N/m
    public float Damping;      // N·s/m
    public float MaxForce;     // N
    public float3 DeviceForce(float3 proxyPosition, float3 proxyVelocity, float3 devicePosition, float3 deviceVelocity);
}

// HAP-6：设备抽象与线程间交换
public interface IApxHapticDevice
{
    bool TryReadPose(out float3 position, out quaternion rotation, out double time);
    void WriteForce(float3 force, float3 torque);
}

public sealed class ApxHapticBridge
{
    public void PublishModel(ApxForceModel model);            // 物理步结束后，主线程
    public ApxForceModel LatestModel { get; }                 // 触觉线程
    public void PublishPose(float3 position, quaternion rotation, double time); // 触觉线程
    public bool TryGetLatestPose(out float3 position, out quaternion rotation, out double time); // 物理步开始前
}
```

### 3.5 训练系统接口

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-13 | 评分事件与指标 | 没有。需要：切口长度、深度、位置，组织损伤量，超力事件，缝合指标（SUT-8） | 未开始 | 事件 API 与一个标准场景的评分输出 |
| MED-14 | 输入录制与回放 | 同一机器、同一构建下结果逐位一致（`SameInputsGiveBitIdenticalResults`）；基准测试输出节点位置哈希。没有录制与回放 API | 部分完成 | 录制器械位姿与操作序列，回放后 `positionsHash` 与录制时一致 |

### 3.6 平台

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-15 | URP 支持 | 只支持内置渲染管线（切面着色器 `ApxTissueInterior.shader`） | 未开始 | URP 下标准场景 S1–S6 截图正常 |
| MED-16 | Player 与 ARM 平台 | 性能数据均在编辑器中测得（见 PERF-4） | 未开始 | Quest（ARM、IL2CPP）上运行演示场景并记录基准数据 |

## 4. 参数化与 AI 可配置

目标：参数以数据形式存在并带元信息（单位、范围、生效时机），可由 AI 通过 schema 读取、通过 MCP 工具修改，并通过 JSON 指标验证修改结果。

| 编号 | 内容 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| CFG-1 | 参数元数据特性，例如 `[ApxParam(Unit = "Pa", Min, Max, Typical, Apply = Hot / Rebuild / Init)]` | 只有 Unity 的 `Range`、`Tooltip` | 未开始 | `ApxSoftBody` 全部公开参数带元数据 |
| CFG-2 | 修正生效时机，提供 `Rebuild()` | 见 BUG-5：`density`、`preStrain`、`skin*` 下一步生效；`resolution` 运行时修改时输出警告，`Rebuild()` 重新体素化并重新取得钉住区域 | 已完成 | `density` 修改后下一步生效；Init 类参数修改时触发重建或输出警告；`Rebuild()` 保留钉住设置 |
| CFG-3 | 从元数据生成 JSON Schema 与参数文档 | 没有 | 未开始 | 生成的 schema 覆盖 CFG-1 的全部参数 |
| CFG-4 | 材料预设（JSON，附文献出处），组件引用预设并局部覆盖 | 没有 | 未开始 | liver、skin、fat 三个预设；预设文件可文本比较 |
| CFG-5 | `GetParams` / `SetParams` 接口 | 没有 | 未开始 | `SetParams` 返回每个参数的结果：已生效、需重建、超出范围 |
| CFG-6 | 只读派生量 | 已有：`CellSize`、`LastStepMilliseconds`、`LastTopologyMilliseconds`、`LastInitializeTimings`、`TotalMass`。缺：子步 dt、稳定性数 μΔt²/(ρh²)（DESIGN 第 3 节） | 部分完成 | 派生量可通过 CFG-5 的接口读取 |
| CFG-7 | 硬编码常量改为参数 | `SleepSteps`、`SleepSpeed`（`ApxSoftBody.cs:108`、`110`）；接触搜索半径 1.8h、最小距离 0.8h（`SoftBodySimulation.cs:475`、`494`、`495`）；扫掠延伸 0.5h / 0.1h（`SoftBodySimulation.cs:583`）；`ContactInterval`（`SoftBodySimulation.cs:74`）；`MaxKinematicFrames`（`SoftBodySimulation.cs:71`） | 未开始 | 常量带 CFG-1 元数据，默认值不变时 `positionsHash` 不变 |
| CFG-8 | unity-mcp 自定义工具：`apx_get_params`、`apx_set_params`、`apx_run_scenario`、`apx_benchmark` | 基准测试可通过 MCP 的代码执行调用 `ApxBenchmark.ScheduleKeepingScenes`；没有自定义工具 | 未开始 | 四个工具可用；长时间任务异步执行并可查询结果 |
| CFG-9 | 文本描述的实验场景（刀路、参数、期望指标），batchmode 运行并输出 JSON | 基准测试输出 JSON，场景固定在代码中；标准场景截图工具（S1–S6）只输出 PNG 与日志 | 部分完成 | 场景文件定义刀路与期望指标；S1–S6 输出 JSON 指标 |
| CFG-10 | 闭环调参示例 | 没有。依赖 HAP-1 的器械受力与 HAP-3 的校验 | 未开始 | 调整材料参数，使压痕的力-位移曲线与给定文献数据的误差小于设定值 |

## 5. 性能与基准测试

| 编号 | 内容 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| PERF-1 | 基准测试框架 | `AdaptorPhysX/Run Benchmarks`、`ApxBenchmark.RunBatch`，网格 box、liver、bunny、dragon，分辨率 16/32/48/64，JSON 输出（DESIGN 第 8 节），结果见 DESIGN 6.1 节 | 已完成 | — |
| PERF-2 | 基准矩阵的其余维度 | 未覆盖：分辨率 96/128（`resolution` 可由代码设置，Inspector 范围为 4–64）；子步 8/32（`-apxSubsteps` 已支持，未运行）；抓取状态；Armadillo、Thingi10K、真实器官分割网格 | 部分完成 | 上述维度各有一组结果 |
| PERF-3 | 指标缺口 | Native 内存只有 Unity 总分配量的差值（编辑器中为近似值）；托管分配为 Mono 堆增长量，期间发生 GC 时记为 -1；首次 JIT 耗时在预热中，未单独记录 | 部分完成 | 记录仿真与表面网格的 Native 内存；首次切割帧耗时单独记录 |
| PERF-4 | batchmode、Player（IL2CPP）与 ARM 数据 | 6.1 节数据来自交互式编辑器（Debug、Jobs Debugger 开启）。batchmode 入口未实际运行过 | 未开始 | batchmode 运行一次；Windows Player（IL2CPP、Release）与 Quest 各一组结果 |
| PERF-5 | 仿真拓扑局部重建 | 每个切割帧全量重建：1428 个体素 1.2 ms，77220 个体素 51 ms（6.1 节） | 未开始 | 77220 个体素时切割帧扫掠小于 5 ms；属于 M3 |
| PERF-6 | 表面更新与渲染拓扑重建 | 龙（43.5 万顶点）每帧表面更新 8–11 ms，切割帧表面重建 p95 200–460 ms；box、liver、bunny 分辨率 64 时 p95 约 160 ms | 未开始 | 切割帧表面重建 p95 小于 16 ms；顶点在 GPU 上写入（M4） |
| PERF-7 | 初始化耗时 | box 64 的 4.78 s 中表面细分 3.67 s（`MeshUtil.Subdivide`，托管 List）；龙 64 的 3.93 s 中体素化 2.43 s（`InsideParityJob` 每行遍历全部三角形）。使用 ECO-7 的 blueprint 时两项均跳过；不使用 blueprint 时仍为上述耗时 | 部分完成 | 两项各降低一个数量级；或由 ECO-7 的 blueprint 缓存替代 |
| PERF-8 | 自适应 lattice | 分辨率上限 64，lattice 均匀 | 未开始 | 与 MED-7 一起验收 |
| PERF-9 | 减少子步数的求解方法 | 16 子步、每子步 1 次迭代。候选：Chebyshev 加速、Vertex Block Descent | 未开始 | 评估报告：相同精度下的耗时对比（与 BUG-3 的体积损失一起比较） |
| PERF-10 | 开放网格的内外判定 | 体素化要求闭合网格（射线奇偶测试）。当前 bunny 为闭合版本，不受影响 | 未开始 | 广义缠绕数判定；原始扫描版兔子（底部有孔）体素化结果正确 |

## 6. 建议顺序

| 顺序 | 条目 | 状态 |
|---|---|---|
| 1 | 基准测试框架（PERF-1） | 已完成 |
| 2 | 切割与接触缺陷（BUG-1、BUG-2、BUG-4、BUG-6），之后 BUG-3 | 部分完成（BUG-1、BUG-4、BUG-5、BUG-6、BUG-7 已完成；BUG-2、BUG-3 原因已确认，修复需改动求解器） |
| 3 | 参数元数据、生效时机、MCP 工具（CFG-1、CFG-2、CFG-5、CFG-6、CFG-8；同时处理 BUG-5） | 部分完成（CFG-2 已完成） |
| 4 | 跨物体碰撞、MeshCollider 与 SDF 器械、局部重建（ECO-1、ECO-2、PERF-5、PERF-6） | 部分完成（ECO-1 部分完成，地面摩擦下的相向挤压测试未通过，见 BUG-8；ECO-2 已完成；同时完成 ECO-3、ECO-6、ECO-8，ECO-7 部分完成） |
| 5 | 力反馈计算接口（HAP-1 至 HAP-7，按编号顺序） | 部分完成（HAP-1 已完成） |
| 6 | 体素材料标签 → 非线性与粘弹性材料 → 撕裂（MED-5、MED-1、MED-2、MED-6） | 未开始 |
| 7 | 缝合：SUT-1、SUT-5、SUT-9 先行；SUT-2 至 SUT-4、SUT-6 依赖穿刺（MED-9）与缝线（ECO-5） | 未开始 |

第 2 项排在参数化之前：BUG-1 与 BUG-2 影响切割结果与基准测试中的“全切结果”列，后续改动需要以此为对照。

## 变更记录

| 日期 | 提交 | 内容 |
|---|---|---|
| 2026-10-26 | `17fde10` | 基准测试框架（PERF-1）；基准测试与代码检查发现 BUG-1 至 BUG-5 |
| 2026-10-26 | `7ee1147` | 建立本文件 |
| 2026-10-26 | — | 新增 3.3 节缝合（SUT-1 至 SUT-9）；力反馈改为计算接口范围（3.4 节，HAP-1 至 HAP-7 与接口草案），替代原 MED-11、MED-12 |
| 2026-10-26 | `8eda481` | BUG-4 已完成：端点在刀面上的棱按有符号距离归到刀面正侧；新增 EditMode 测试；基准测试 `positionsHash` 与修复前一致 |
| 2026-10-26 | `db3ad63` | 演示：刀刃深度从刀下组织表面算起，拖动时随表面高度变化。新增 BUG-6；BUG-1 补充证据 |
| 2026-10-26 | `44aa786` | BUG-2 原因已确认，记录求解顺序与镜像剖分实验（未采用）；新增并完成 BUG-7（旋转放置的物体变形）；BUG-1 补充证据 |
| 2026-10-27 | `c6e5da1` | BUG-1 已完成（刀路内节点侧别）；BUG-5、CFG-2 已完成（参数下一步生效、`Rebuild()`）；BUG-3 原因已确认；BUG-2 补充对称 Gauss-Seidel 实验 |
| 2026-10-27 | `1e35fa0` | 与 Obi 的差距：ECO-1、ECO-2、ECO-3、ECO-6、ECO-8、HAP-1 已完成，ECO-7 部分完成（blueprint），PERF-7 部分完成；CFG-6 增加 `TotalMass`。新增 EditMode 测试 10 个（`InteractionTests`）、交互演示场景与标准场景 S8–S11。默认设置下基准测试 16 组的 `positionsHash`、块数与断棱数与 `c6e5da1` 一致 |
| 2026-10-27 | `8133858` | BUG-6 已完成：断开刀尖、刀跟外侧的表面外棱；皮肤顶点的副本按副本边界断棱判断侧别，同一切口的平面按三角形处的距离去重；新增 EditMode 测试；基准测试 `positionsHash` 与不启用规则时一致 |
| 2026-10-27 | — | ECO-2 改为部分完成：MeshCollider 与 `ApxDistanceField` 资产路径没有 EditMode 测试，验收改为对应测试；修正过期的文件行号（以 `20c1e5e` 为准） |
| 2026-10-27 | — | ECO-1 改为部分完成：验收中的相向挤压没有测试（`StackedBodiesRestOnEachOther` 只覆盖静止堆叠），验收改为分列已覆盖与未覆盖两部分 |
| 2026-10-27 | — | ECO-2 已完成：新增 `MeshColliderAndFieldAssetFloorsHoldTheBodyLikeABoxCollider`（MeshCollider 与 `ApxDistanceField` 资产地面，最低节点高度与 Box 碰撞体之差 0.000 mm）。新增 ECO-1 相向挤压测试 `PinnedBodyPushesAnotherWithoutPassingThrough`，未通过，标为 `[Ignore]`；新增 BUG-8（持续挤压时软体之间相互穿透）。运行时代码未改动 |
| 2026-10-27 | — | BUG-8 原因已确认：步内对方副本不更新、按份额推出与每子步 0.25h 的上限，使持续载荷下的残余重叠随载荷增大；越过 SDF 精确带或位于对方 SDF 中轴（梯度为 0）的节点不再被检测。`PinnedBodyPushesAnotherWithoutPassingThrough` 的穿入深度改为按物体间接触的定义计算（原几何深度在角节点上偏大）。运行时代码未改动 |
| 2026-10-27 | — | BUG-8 部分完成：相互接触的物体同步推进子步（`SubstepGroup`，每个子步求解后刷新对方副本），本步不推进的对方作为静止障碍；物体间接触另用补全深处的格点 SDF（`ContactSdfJob`）。新增 `SqueezedBodiesKeepTheirContactUnderASustainedLoad`、`PinnedBodyPushesAnotherAlongAFrictionlessFloor`；地面摩擦下的推动仍穿透（被推物体倾倒，受力集中在推动体底边），ECO-1 验收测试保持 `[Ignore]` |
