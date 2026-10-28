# 待办清单

本文件列出与 Obi 生态、医疗仿真需求相比尚缺的功能，以及基准测试与代码检查中发现的缺陷，并逐项对照当前实现。
每个条目有固定编号，新对话中直接引用编号（例如“处理 BUG-2”）。

- 建立时的对照基准：提交 `17fde10`（2026-10-26），分支 `dev/softbody`；各条目的现状随之后的提交更新，最后一次全面核对为 2026-10-28（`f5cc1d5`，同时对照 Obi 7.2 的 solver、后端接口、softbody、附着与抓取组件、rope 的 pinhole、blueprint 编辑器）。文件行号以 `0ba4a57` 为准。
- 里程碑编号（M1–M7）见 `docs/ROADMAP.md`；算法与数据结构见 `docs/DESIGN.md`。

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
     运行结束时自动与 `Benchmarks/baseline.json`（ENG-3）比较，日志与结果 JSON 的 `baseline` 字段列出不一致的组。
     改动有意改变仿真结果时，用 `WriteBaseline = true`（batchmode 为 `-apxWriteBaseline`，菜单为
     `Run Benchmarks and Write Baseline`）重写基线，并在提交中写明哪些组改变及原因。
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
| BUG-8 | 持续挤压时软体之间相互穿透 | 原因（2026-10-27 确认）：(1) 对方副本每步复制一次、按份额推出（`PartnerShare`）、每子步推出量上限 0.25h，使持续载荷下本物体弹性推入的部分不能完全推出，残余重叠随载荷增大；(2) 节点越过对方 SDF 的精确带（1.5 个体素，更深处取 −2h）后梯度小于 0.5，不再被检测；(3) 对方 SDF 中轴处梯度为 0，同样不被检测。已修改（DESIGN 9.4 节）：相互接触的物体同步推进子步（`SubstepGroup`），每个子步的约束求解之后各物体把副本刷新为对方求解后的位置，两个物体看到同一重叠、各推出自己的份额；本步不推进的对方作为静止障碍；接触另用精确带以外补全的格点 SDF（`ContactSdfJob`）。结果：静态挤压（`SqueezedBodiesKeepTheirContactUnderASustainedLoad`）最深节点 4.43 mm，停住期间不增加，节点速度 0.001 m/s（修改前由 4.5 mm 增至 6.2 mm，节点速度 1.4 m/s）；无摩擦地面上的推动（`PinnedBodyPushesAnotherWithoutPassingThrough`，ECO-1 验收）最深节点 4.33 mm，被推物体前移 75.5 mm；`StackedBodiesRestOnEachOther` 静止后最大节点速度 0.007 / 0.003 m/s（修改前 0.020 / 0.013），下方块休眠。未解决：被推物体放在 0.6 摩擦地面上时（`PinnedBodyPushesAnotherAlongAFloorWithFriction`，标为 `[Ignore]`；原为 ECO-1 的验收场景），被推物体向前倾倒（推动第 15 帧顶部前移 16.9 mm、底部 9.0 mm，背面底部抬起约 4 mm），接触集中在推动体底部两行节点，这里对方没有节点进入推动体，推动体只推出自己的份额，残余重叠在推动第 9 帧为 1.6–2.1 mm 并继续增大，被推物体前移 5 mm 后停下，推动体穿过（最深 27.6 mm）。试过但未采用：中轴处改用副本角点的最小二乘梯度（堆叠与静态挤压中节点速度 0.6–0.9 m/s）；推出量上限放宽到 0.5h（该场景最深 8.8 mm、被推物体前移 47 mm，但静态挤压中节点速度 1.5 m/s）；由本物体推出自身超出预测部分、不分份额（动量不守恒，堆叠时节点速度 0.16–0.65 m/s）。另外试过三个候选方案（2026-10-27，未采用，代码已撤回）：(a) 反作用：节点推出时记录对方表面点所在的四面体，由一个单线程 job 把 −w b λn 累加给对方的角点，对方在下一个子步施加；面对面接触时对方节点也在接触中，跳过。两个 lattice 对齐时推动体的面节点落在对方第二层节点上，按节点所在四面体判断会误判为单侧，要改用节点在对方表面上的投影点所在的四面体。(b) 推出量上限加上节点本子步向对方移动的距离。结果（静态挤压最后 30 帧最大节点速度 / 摩擦推动最深节点 / 被推物体前移）：已提交版本 0.001 m/s / 27.6 mm / 5.2 mm；(a) 0.678 m/s / 12.5 mm / 50.5 mm；(b) 1.200 m/s / 14.1 mm / 47.2 mm；(a)+(b) 1.217 m/s / 7.5 mm / 50.8 mm。无摩擦推动均为 4.3 mm。只要接触更强，两端钉住的静态挤压就出现 0.7–1.2 m/s 的抖动。使用反作用时被推物体能被推走，但它在摩擦地面上翻倒（推动 35 帧后顶部前移 83 mm、底部前移 38 mm，背面底部抬起 26 mm），接触始终集中在推动体底边；第 26 帧起底边节点到达对方背面与底面之间的中轴（距两面各 12.5 mm），梯度为 0，不再被检测。改变几何的结果（已提交版本，0.6 摩擦；最深节点 / 被推物体重心前移 / 背面顶行比底行多前移）：被推物体 6 cm，推动体中心高 35、30、25 mm 时为 27.6 / 5.2 / 2.8、27.5 / 6.7 / 3.2、27.6 / 8.0 / 2.7 mm；被推物体沿推动方向加长到 8 cm（中心高 35、30 mm）为 27.7 / 1.6 / −2.7、27.7 / 1.9 / −2.6 mm，加长到 10 cm（30 mm）为 27.7 / 1.9 / −0.3 mm。被推物体不倾斜时也几乎不滑动，推动体照样穿过：摩擦下的失败不只是倾倒引起的，接触为何在运动中传不出克服地面摩擦所需的力尚未查明。 | 部分完成 | 查明摩擦地面上被推物体为何几乎不滑动：记录推动中两物体之间的接触力（按推出量换算）与被推物体受到的地面摩擦力随时间的变化，与无摩擦时对比；之后再考虑每个子步多次"刷新—接触"迭代、反作用传递与中轴处的检测（试验结果见左） | `PinnedBodyPushesAnotherAlongAFloorWithFriction` 去掉 `[Ignore]` 后通过；`PinnedBodyPushesAnotherWithoutPassingThrough`、`SqueezedBodiesKeepTheirContactUnderASustainedLoad`、`StackedBodiesRestOnEachOther` 仍通过；S8 中接触的两个物体没有抖动；单物体基准测试 16 组的 `positionsHash` 不变 |
| BUG-9 | 刀刃停在表面与表面外节点之间时不分离 | 表面外棱的断开规则（BUG-6，DESIGN 4.1 节）只作用于刀尖、刀跟外侧在本步扫过的范围；刀刃前方的表面外棱不断开：刀刃越过表面、但在到达表面外的 lattice 节点之前停下时，该处边缘仍连在一起（DESIGN 第 7 节）。BUG-1 排查中把规则扩展到刀刃前方（交点在刀刃前方、距扫掠四边形不超过静止 SDF 时也断开）后，单节点碎块增多（bunny 64 由 23 块增至 33 块），未采用 | 原因已确认 | 设计刀刃前方表面外棱的断开条件，且不产生单节点碎块。候选：只断开与本刀路已断开的棱相邻的表面外棱；或在刀路结束（`ApxScalpel` 停止切割）时补断刀刃前缘与表面之间的棱 | 新增 EditMode 测试：box 分辨率 8，竖直刀刃沿 x 全切，刀刃前缘停在越过对侧表面 0.2h 处（未到达表面外节点），物体分成两块；基准测试 16 组的块数不变，碎块数不增加 |
| BUG-10 | 开放网格初始化时没有提示 | 已修复。此前 `ApxSoftBody.Initialize` 的报错信息要求“readable closed mesh”，但只检查 `mesh.isReadable`，网格有孔时经过孔的格点行内外判定错误（DESIGN 第 2 节），组件不输出任何提示；开放棱统计只在基准测试的编辑器程序集中。修复：开放棱统计移到运行时程序集（`MeshUtil.CountOpenEdges`，Burst），`ApxSoftBody` 在体素化之前（不使用 blueprint 的 `Initialize`、`BakeBlueprint`）统计细分前网格焊接后的边界棱，大于 0 时输出警告；基准测试改用同一函数，统计结果不变。非流形棱不警告（龙的 OBJ 有 108 条，体素化正常）。测试模型中扫描版兔子（`Stanford_Bunny_69451_triangles.fbx`）有 223 条边界棱，龙与 Fertility 的 FBX 为 0。支持开放网格见 PERF-10。补充（2026-10-28）：此前按位置精确焊接，`MeshUtil.CreateEllipsoid` 生成的椭球接缝顶点（θ = 0 与 2π）只差约 1e-8 m，被判为 142 条边界棱，演示中的 Liver Lobe、Dropped Lobe 因此输出警告并改用缠绕数判定（PERF-10）；改为焊接距离小于包围盒对角线 1e-6 倍的顶点（`MeshUtil.WeldTolerance`）后为 0。龙的 OBJ 的非流形棱统计随之由 108 变为 697（扫描网格中有相距不到 0.2 µm 的顶点），边界棱仍为 0 | 已完成 | — | `OpenMeshWarnsOnInitialize`：去掉一个三角形的立方体网格有 3 条边界棱，初始化时输出警告（`LogAssert.Expect`），闭合网格为 0、不输出；基准测试 16 组与基线（ENG-3）一致；`MeshUtil.CreateEllipsoid` 生成的椭球为 0 条（接缝顶点按容差焊接） |
| BUG-11 | 手动切割的切口呈锯齿状 | 人工试用 ENG-5 的手术刀时发现（2026-10-27，交互演示的 Hammock，体素 5.7 mm，受自重与钢球拉伸）：切口为锯齿，切面上有翻出的小片。用脚本回放指针复现（相机斜视、每帧 2 个固定步、指针左右抖动 2 mm、刀深 33 mm），原因有三：(1) 指针所在的水平面随刀下组织高度变化，切口张开或刀越过组织边缘时表面下降，斜视的指针沿视线方向移动，刀一帧跳过 40 mm；(2) 指针的抖动直接成为刀路，刀路在一个体素内来回转弯；(3) 刀路转弯时一半以上的断棱由刀路侧别断开、且不与本次扫掠三角形相交，这类棱的切口点取棱中点，切面顶点偏离刀路最多半个体素。修复：指针在过按下点的水平面上移动，整个刀路不变；刀以稳定绳跟随指针（`ApxScalpelTool.stabilizer`，默认 8 mm，`[` / `]` 调整，刀路滞后指针一个绳长）；侧别断开的棱按两端记下侧别时到刀面的距离求切口点（`StrokeDistance`、`CutJob.LabelCrossing`，DESIGN 4.1、10.2 节；切口点只用于渲染）。结果（同一回放）：切口点在静止空间中偏离刀路直线的均方根 2.30 → 1.50 mm，最大 8.96 → 4.86 mm，偏离超过一个体素的 19 → 0 个，切口目视平整；Wet Tissue 半切（体素 5 mm）均方根 1.15 → 0.44 mm。只改切口点（不用绳、竖直指针）时最大 5.91 → 4.01 mm、超过一个体素的 24 → 0 个，切口仍随抖动呈锯齿。标准场景截图中有切口的 S2、S3b、S5b、99 与 D1、D3 随之变化（S3b 切面上沿与下方块切口边缘的锯齿消失），其余不变 | 已完成 | — | `WavyStrokePutsTheCutPointsOnTheBladePath`：振幅 3 mm、波长 45 mm 的刀路（体素 6 mm），切口点距刀路不超过 0.15h（实测 0.40 mm，取棱中点时 3.10 mm）；`ScalpelToolStringKeepsHandJitterOutOfTheCut`：指针左右抖动 2 mm 时刀路横向偏移小于 1 mm（实测 0.80 mm，不用绳时 2 mm），全切为两块；基准测试 16 组 4 个阶段的 `positionsHash`、块数与断棱数与修改前一致 |
| BUG-12 | 刀路急转弯时断开刀未经过的棱 | 已修复。原因（2026-10-27 用刀路回放与节点侧别的记录确认）：(1) 每一步的扫掠四边形沿本步运动方向向后延长 0.5h，刀路转弯时延长部分离开刀路；(2) 夹在两种侧别之间的无侧别节点按本步刀面补记侧别，刀刃前方的节点也补记：box 0.12 × 0.05 × 0.1 m、分辨率 20（h = 6 mm，无重力），竖直刀刃沿 z 每步 3 mm、x 方向振幅 2.5 mm、波长 15 mm（转弯半径约 2.3 mm）时，节点 (−3, −47) mm 在刀刃前方 3.9 mm 处按一段斜向刀面被记到 x 正侧，刀路随后在 x = −2.5 mm 处折返，该节点实际在负侧，它通向两侧的 x、z 向棱被断开，切口点距刀路最远 4.2 mm。修复：每一步另扫一个尾部四边形，从上一步结束时的刀刃沿上一步的运动方向向后 0.5h，即重扫上一步刀路的末端，本步四边形不再向后延长，两个四边形各用自己的平面；节点按投影所在的最近扫掠三角形记侧别；刀刃前方的节点不补记侧别，等刀经过时再记（`CutJob`、`SoftBodySimulation.Cut`，DESIGN 4.1 节）。只要求补记的节点投影在扫掠面上时，bunny 64 全切不再分开（被刀带动、投影落在扫掠面外的节点不补记），因此只排除刀刃前方的节点。基准测试：16 组块数不变；12 组断棱数不变，bunny 48、bunny 64、dragon 48、dragon 64 为 1524 → 1527、2835 → 2866、214 → 213、696 → 690，两大块为 17998 / 13798、46360 / 23184、7207 / 5530、20249 / 5875（修改前 17995 / 13801、46687 / 22856、7206 / 5530、20252 / 5872），碎块不增加；补记推迟到刀经过时，16 组切割与切割后两阶段的 `positionsHash` 全部改变（静止、落下两阶段不变）。标准场景截图中有切口的 S2、S3、S3b、S4、S5、S6b、99 与 D1、D3 有细微变化，未见碎块或切口变差 | 已完成 | — | `SharplyTurningStrokeSeversOnlyTheEdgesItCrosses`：上述波长 15 mm 的刀路全切为两块，断开的棱距刀路都不超过 0.1h（各步向前延长 0.1h），切口点距刀路不超过 0.2h（修复前有刀未经过的棱断开，切口点最远 4.2 mm）；`WavyStrokePutsTheCutPointsOnTheBladePath` 增加同样的断棱检查；`FullCutOfABodyLyingOnAColliderSeparates` 仍通过；基准测试 16 组块数不变，碎块不增加 |
| BUG-13 | 不规则网格落地后少数轻节点持续抖动，物体不能休眠 | 真实网格摸底中发现（2026-10-28，DESIGN 6.2 节）：基准测试落下 90 步后的最大节点速度，bunny 32 / 48 / 64 为 1.21 / 2.67 / 3.58 m/s，dragon 48 / 64 为 1.24 / 2.04 m/s，扫描版兔子与 Fertility 48、64 同一量级；box、liver 不超过 0.17 m/s。bunny 48 落地后 150 步，31796 个实节点中有 934 个相对平均速度超过 0.05 m/s，最快的是 0.5–1.4 mg 的轻节点（填充率低的边界体素的角点），不在地面上（高于地面 7–19 mm）；自由下落时没有。对照（同一场景，第 150 步，相对平均速度的最大值、超过 0.05 m/s 的节点数）：默认 2.40 m/s、934 个；摩擦 0 时 3.33 m/s、2227 个；阻尼 10 时 3.46 m/s、1165 个；32 子步 1.35 m/s、59 个；关闭 `selfCollision` 0.49 m/s、24 个。物体只有一块，因此起作用的是同一块内静止距离大于 1.8h 的折叠自接触（DESIGN 3.1 节）：自重下形变后，相对的两片表面上的表面外节点相距小于 0.8h 而被推开。休眠阈值为 2 mm/s，这些物体始终不休眠，每帧都在计算；靠近这些节点的皮肤顶点可能随之抖动（未目测确认）。2026-10-28 查明：活动接触对两端的静止距离约 14h、当前距离约 0.7h，是兔子的耳朵等部位在自重下下垂（E = 30 kPa）后贴到身体上的真实折叠；起作用的多是表面外约 1.1h 的轻节点，按节点之间 0.8h 推开，两片实际表面之间留着约 3h 的间隙。自包含的复现：C 形物体（下臂、背板、上臂，上臂 0.1 × 0.02 × 0.06 m，两臂间隙 20 mm，三个盒子按缠绕数体素化，分辨率 20）放在地面上，上臂在自重下下垂压到下臂：240 帧后 3864 个节点中约 1100 个超过 0.05 m/s，最大 0.79 m/s；上臂 x > 0 的 924 个节点中 559 个已低于下臂顶面（关闭 `selfCollision` 时内部节点降到 y = 6.3 mm，开启时 7.0 mm，几乎没有挡住），3783 个接触对处于上臂节点已到下臂节点下方的交叉状态。原因：节点对接触只按距离沿两节点连线推开，不判断侧别；载荷下节点互相穿过之后，接触把它们往错误的方向推，穿透被锁住，节点在穿透区域里来回推挤（每 2 个子步一次 Jacobi 修正，松弛系数 0.8）。试过（未采用，代码已撤回）：同一块的接触距离减去两节点在表面外的部分（`0.8h − max(s_i, 0) − max(s_j, 0)`），C 形物体超过 0.05 m/s 的节点约 500 个、最大 0.61 m/s，bunny 48 为 528 个、1.22 m/s，上臂照样穿过。另一个原因（2026-10-28 查明并修改）：关闭 `selfCollision` 后剩下的 23 个节点（最大 0.49 m/s）是只属于一个填充率为下限 1/8 的体素的角点，质量只有体素质量的 1/64，同一四面体的其他角点可达一个体素的质量；物体形变后它们在相邻子步之间来回运动（每帧末的位置几乎不变，32 子步时没有，阻尼 10、ν = 0.3 时仍有）。节点质量加下限（体素质量的 1/16，DESIGN 第 3 节）后关闭 `selfCollision` 时为 0 个、最大 0.002 m/s，bunny 48 总质量增加 0.45%。2026-10-28 修改折叠接触（DESIGN 3.1 节）：同一块不再按节点对处理；外两层节点（静止 SDF 不小于 −0.75h）推出同一块中静止时不在其周围 4 × 4 × 4 个体素内的副本，副本角点的静止 SDF 在形变四面体上线性插值，沿梯度推出到两个表面接触（深度 −φ − s，不除以梯度长度），每个子步一次（`FoldContactJob`），松弛系数 0.5，每子步不超过 1h；候选副本每帧按节点相对副本在本步内的路径筛选、按到节点的距离排序（`FindFoldCandidatesJob`）。不同块的节点对接触不变。结果：C 形物体 240 帧后上臂中位于自身表面以内的节点（静止 SDF ≤ 0）低于下臂顶面的 5 / 924 个（关闭 `selfCollision` 时 325 个；修改前上臂穿过下臂，任一上臂节点低于下臂顶面的 643 个），超过 0.05 m/s 的节点 0 / 3864（修改前 1103），最大 0.000 m/s（修改前 0.80），第 130 帧满足休眠条件；bunny 48 落地 150 步相对平均速度超过 0.05 m/s 的实节点 0 个（修改前 941、只加质量下限时 913），最大 0.002 m/s（修改前 2.15），第 186 步满足休眠条件（实节点最大速度连续 30 步低于 2 mm/s）；bunny 32、dragon 48 为第 152、122 步。接触测试与其余 EditMode、PlayMode 测试通过。耗时：分辨率 64 的 box、bunny 静止步约多 5 ms（11–13%，主要为每帧的候选查找）。未满足：基准测试中 bunny 64 与 dragon 64 的全切块数改变（11 → 6、2 → 4）。bunny 64 在 E = 30 kPa 下落地后倾倒、耳朵摊开（高 82 mm），刀面只经过一侧，只断开 641 条棱（基线 2866），剩一大块（67889 / 1494 / 164 / 4）；只加质量下限时刀穿过物体但仍为一大块（69541 / 3 / 2 / 1），修改前刀经过时节点速度达 4.5 m/s、结果分成两块。bunny 64 第 120 帧后整体静止，但 7–59 个耳朵上的节点继续以 0.15–2.3 m/s 运动，600 帧内不休眠。dragon 64 仍分成两大块（14006 / 12111），另有 6、1 个节点的碎块 | 部分完成 | 查明 bunny 64 落地后耳朵持续运动的原因（耳朵与地面、身体的接触，或分辨率 64 下的轻节点）；基准测试的全切对落地后倾倒、摊开的物体选取刀面的方式（DESIGN 第 8 节）；dragon 64 全切的两个碎块 | C 形物体在地面上 240 帧后上臂节点不进入下臂（低于下臂顶面的少于 1%），超过 0.05 m/s 的节点少于 1%、最大小于 0.1 m/s；bunny 48 落地 150 步后相对平均速度超过 0.05 m/s 的实节点少于 50 个、最大值小于 0.1 m/s，物体能休眠；`CutOffPieceRestsOnTheRestInsteadOfFallingThrough` 等接触测试仍通过；基准测试 16 组的块数不变 |
| BUG-14 | 高分辨率下 Fertility 全切不分离 | 基准测试（DESIGN 6.3 节）：Fertility 的 FBX 分辨率 128（体素 1.17 mm）全切断开 5454 条棱，结果为一块 152074 个实节点加 4 个不超过 5 个节点的碎块；分辨率 96 时分成 46299 / 23406 两块，16–64 时也都分开。落下阶段结束时最大节点速度 3.1 m/s（BUG-13 的抖动），刀推进约 60 帧，刀路侧别在刀推进 8h（约 9 mm）后失效（DESIGN 4.1 节），可能与 BUG-1 相同：组织在刀经过一条棱两端的时刻之间移动 | 原因未查明 | 记录未断开、两端在刀经过时分处刀面两侧的棱（BUG-1 的方法），看它们的位置与两端的侧别；对照：关闭 `selfCollision`（排除 BUG-13 的抖动）、刀推进更快（帧数减半）、刀面平移半个体素 | Fertility 128 全切分成两大块，碎块不增加；基准测试 16 组的块数不变 |
| BUG-15 | 第二刀经过已张开的切口时，切口壁出现空洞与碎切面，皮肤跨过切口 | 已修复。用户试用交互演示时发现（2026-10-28，Hammock，体素 5.7 mm）：第一刀切口完整，第二刀之后切口壁上出现空洞与碎切面。在演示场景中复现：Hammock 在距右夹子 3 cm 处切穿，切开的两部分在钢球与自重下分开后，第二刀沿原线重切、平行一个体素、摆动、反向斜切或斜交；计量穿过切口区域的随机直线与渲染表面相交次数为奇数的比例（洞与开放的翻片）与长于 2h 的皮肤三角形。原因（修复前的数据来自 `39708d1`，单刀 1.1 %、0 个）：(1) `SurfaceMesh.FitPlanes` 只按法线方向给断棱分组，第二刀与第一刀平行且落在同一体素时，两刀的断棱拟合成一个位于两刀之间的平面，皮肤按它拆分并选副本，分到切口另一侧：平行一个体素 13.9 %、63 个，沿原线重切 3.3 %、23 个；(2) 皮肤块的每个顶点各在自己的体素中按平面判断侧别，切口在体素内转折时（第二刀摆动、斜交）同一块的顶点落在切口两侧的不同块上：反向斜切 7.0 %、36 个，斜交 30° 3.0 %、8 个；(3) 切口壁在两刀之间已经移动，第二刀擦过壁面，削下 1–12 个节点、比一个体素还薄的薄片：摆动重切 5 个 4 节点碎块，反向斜切 12、4、2、2、1 节点各一。修复：(1)(2)（`64038e9`，DESIGN 第 5 节）断棱按所隔的两个副本与法线方向分组拟合平面；同一皮肤块的顶点必须用相连的副本求值（同一体素同一副本，相邻体素共享节点；`SurfaceMesh.Together`，托管与 Burst 两条路径相同）；(3)（DESIGN 4.1 节）刀路开始时记下已有断棱（`StrokeStartSevered`），一步扫掠在已有切面旁切出少于 16 个实节点的部分时，恢复这一步切断的、一端在该部分上的棱，并在本刀路内保留（`CutJob.KeepChips`、`StrokeKept`，含在状态快照中）；只由本刀路切出的部分不受影响。结果（演示场景，`2bcea79` 之上，单刀 1.0 %、0 个）：沿原线重切 1.1 %，摆动重切 2.0 %、无碎块，反向斜切 1.9 %、无碎块，平行一个体素 1.3 %，斜交 30° 1.7 %，平行两个体素 1.2 %，均没有长于 2h 的皮肤三角形。第二刀擦过切口壁时壁上仍有细缝，见 BUG-16。单刀的基准测试 16 组与基线一致；标准场景截图中 99、D3、S2、S3b、S5b 有不超过 53 个像素的变化，其余不变 | 已完成 | — | `RecutTests`（Hammock 切穿后的第二刀，无钢球）：倾斜 15° 靠向切口、沿切口摆动、反向斜切三例，没有长于 2h 的皮肤三角形（倾斜一例修复前 3 个），不新增少于 8 个节点的碎块（摆动、斜切两例修复前 8 个、6 个），摆动与斜切两例奇数直线的比例比第一刀后增加不超过 2 个百分点（0.2 % → 1.5 %、0.7 %；修复前 3.0 %、2.8 %）；单刀的基准测试 16 组与基线一致 |
| BUG-16 | 第二刀擦过已有切口的壁面时，壁上留下细缝 | 原因（2026-10-28，演示场景探针）：第二刀在离旧切面不到一个体素处顺着壁面切过，切出一边仍相连的薄片，薄片各自转动、错开，壁的下部裂成几块，之间露出背景。刀刃倾斜 15° 靠向切口时，第二刀的 91 条断棱（按体素计）全部距旧切面 0.6h 以内、与旧切面平行（夹角余弦 ≥ 0.995），且没有一条分离出任何部分；穿过切口区域的直线与渲染表面相交次数为奇数的比例 1.6 %（单刀 1.0 %）。切面多边形本身没有新增开口（开口棱 24 → 21 条），缝来自薄片的运动。距旧切面不到 0.9h 的第二刀断棱可以按“是否分离”完全分开：沿原线重切、平行一个体素切下的条带中，这类棱都参与分离。切割时按距离跳过这些棱（0.6h、0.9h）会使平行一个、两个体素的条带、斜交 30° 与斜向汇合的部分都切不下来，未采用。修复（DESIGN 4.1 节）：刀路结束时（`ApxScalpel` 停止切割时调用 `SoftBodySimulation.EndStroke`，新刀路开始时也先结束上一个）恢复本刀路断开的、两端仍在同一块并且贴着已有切面的棱：棱一端的节点上有刀路开始前已断开的棱，法线夹角余弦 ≥ 0.8，该节点位于两交点之间，两交点沿旧法线相距 < 0.75h（`HealStrokeJob`）。结果（演示场景）：倾斜擦壁 1.6 % → 1.0 %（恢复 27 条），反向斜切 1.9 % → 1.4 %（3 条）；沿原线重切、平行一个与两个体素、斜交 30°、斜向汇合，以及从切口垂直、45° 切入的半切都不恢复，结果不变。单刀的基准测试 16 组与基线一致。剩余：摆动重切的刀路以 40–60° 切进壁里，留下的缺口不在夹角条件之内（2.0 %；缺口处的切面有开口棱，缺口旁贴着体素棱的切口壁按 BUG-17 的条件 (2) 不合并副本，仍是角点周围的两个三角形）；条件放宽到 0.5 时摆动与反向斜切都为 1.0 %，但从切口 45° 切入的半切在交接处被恢复 9 条（1.2 % → 1.6 %） | 部分完成 | 区分刀路在该处是沿着旧切口走还是离开旧切口（例如按刀路在该处的运动方向与旧切面法线的夹角），只对沿着走的刀路放宽夹角条件 | `RecutTests`：倾斜 15° 靠向切口的第二刀之后奇数直线的比例比第一刀后增加不超过 2 个百分点（0.2 % → 0.2 %，修复前 3.3 %）；平行一个体素的第二刀照常切下条带（`SecondStrokeOneCellBesideAnIncisionCutsAStripOff`）；从切口垂直与 45° 切入的半切在刀路结束后不恢复任何断棱（`StrokeFromAnIncisionIntoAPartKeepsItsCut`、`SlantedStrokeFromAnIncisionIntoAPartKeepsItsCut`）。剩余部分：演示场景中摆动重切之后与单刀相同 |
| BUG-17 | 切口贴着一条体素棱经过时，切面上有三角形小孔 | BUG-16 排查中发现（2026-10-28）：演示场景 Hammock 单刀切穿后，右块切口壁底部有几个三角形小孔；按静止位置与所属块焊接切面顶点后，只属于一个三角形、两端都在表面以下的切面棱有 24 条，在 12 个体素中，总长 16h。这些体素中切口几乎经过体素的一条棱（角点 1–5，与刀路平行）：通向它的两条 x 向棱在 t = 0.95 处断开，这条 z 向棱也被断开（t = 0.05–0.34），切口右侧的两个角点分成两个各含一个角点的副本（同属右块），各自的切面只是角点周围的三角形；左侧副本的切面是完整的四边形，右侧两个三角形之间留下三角形的孔。修复（DESIGN 第 5 节，托管与 Burst 两条路径相同）：同一块的两个副本之间，若体素内连接它们的断棱都几乎落在自身切面内（法线与棱方向的夹角余弦的绝对值 < 0.3），合为一个区域，按区域生成切面，交点用其棱上实角点所在的副本求值，拟合平面时区域内部的断棱不参与（`SurfaceMesh.CutRegions`）。另有两个条件：(1) 两个副本之间还有穿过切口的断棱时不合并。只要求“有一条切面内的断棱”时，斜 5° 的半切穿过 lattice 线处切口两侧的副本被合并，那一列体素两面的壁都不画（切面面积为两面壁的 0.875）；(2) 共用这条棱的其他体素中，棱两端所在的两个副本之间有穿过切口的断棱时也不合并（`SurfaceMesh.SeparatedNearby`：后一刀从两个节点之间经过，棱上只保留第一刀的交点与法线），否则相邻体素的切面在共用的面上对不上，只用 (1) 时摆动重切后开口棱由 16 条增至 28 条。结果（演示场景，与 `1959136` 相比）：单刀开口棱 24 → 0，奇数直线的比例 1.0 % → 0.8 %；第二刀沿原线重切、倾斜擦壁、平行一个与两个体素、斜交 30°、斜向汇合，以及从切口垂直、45° 切入的半切之后均为 0 条，奇数直线的比例降低 0.1–0.3 个百分点；摆动重切 16 条（9 个体素，与修复前相同，2.0 %）、反向斜切 22 → 16 条（1.4 %），都在第二刀切出的缺口处，见 BUG-16。基准测试 16 组与基线一致；标准场景截图见变更记录 | 已完成 | — | `SoftBodyTests.CutAlongLatticeEdgesLeavesNoHoleInTheWalls`：沿一排 lattice 线断开与切面平行的 z 向棱后切穿，切面没有开口棱（修复前 352 条），托管与 Burst 重建结果相同；`SlantedPartialCutKeepsBothWalls`：斜 5° 的半切，切面面积不少于两面壁的 0.95（修复前 1.000，不加条件 (1) 时 0.875） |

## 2. 与 Obi 生态的差距

| 编号 | 能力 | Obi | AdaptorPhysX 现状 | 状态 | 验收 |
|---|---|---|---|---|---|
| ECO-1 | 不同物体之间的碰撞 | 同一 solver 内全部物体互相碰撞 | 已实现（DESIGN 9.4 节）：每轮步进开始时复制对方附近的体素副本，节点按对方静止 SDF 在形变四面体上的插值推出，按质量分担，`collideWithBodies` 与 `collisionLayers` 过滤；相互接触的物体同步推进子步，每个子步刷新副本（BUG-8）。`StackedBodiesRestOnEachOther`：小块停在大块上（中心 57.6 mm，关闭接触时 18.4 mm）；S8 中肝叶停在组织块上；`SqueezedBodiesKeepTheirContactUnderASustainedLoad` 通过。限制：切面不在对方 SDF 中；相互接触的两个物体通常只有一个休眠；被推物体放在摩擦地面上时推动体穿过对方（BUG-8，`PinnedBodyPushesAnotherAlongAFloorWithFriction`，标为 `[Ignore]`） | 已完成 | `PinnedBodyPushesAnotherWithoutPassingThrough`：一个物体由钉住区域（`pinColliders`）以 0.1 m/s 推向放在无摩擦地面上的另一个物体，被推物体能平移；挤压期间任一节点穿入对方表面的深度不超过 1h（深度按物体间接触的定义：对方静止 SDF 在形变四面体上插值，除以梯度长度），被推物体重心沿挤压方向位移大于 1h；关闭物体间接触时推动体进入对方超过 1h、被推物体位移小于 1h。结果：最深节点 4.33 mm（h = 5 mm），被推物体前移 75.5 mm；关闭接触时 27.7 mm、0 mm。验收原为地面摩擦 0.6，该场景改列 BUG-8 |
| ECO-2 | 碰撞体类型 | 支持 MeshCollider 与距离场 | 已实现（DESIGN 9.1 节）：MeshCollider 运行时按网格烘焙距离场；`ApxDistanceField` 资产（编辑器中烘焙）经 `ApxColliderMaterial` 用于任意 Collider。测试：`DistanceFieldMatchesTheBoxDistance`（距离场本身）、`DistanceFieldFloorHoldsTheBodyLikeABoxCollider`（直接构造的距离场地面）、`MeshColliderAndFieldAssetFloorsHoldTheBodyLikeABoxCollider`（组件路径 `TryMakeFieldShape`：MeshCollider 运行时烘焙；`ApxDistanceField` 资产经序列化复制后使用，Collider 比资产网格高 50 mm，只用于范围查询）。S10 环形器械（MeshCollider 与 `Ring Field` 资产）压入组织，下降量 4.4 mm | 已完成 | `MeshColliderAndFieldAssetFloorsHoldTheBodyLikeABoxCollider`：10 cm 立方体（分辨率 8）落到 MeshCollider 地面、`ApxDistanceField` 资产地面上，最低节点高度与 Box 碰撞体之差均为 0.000 mm（要求小于 0.5 mm） |
| ECO-3 | 与刚体的双向耦合 | 有 | 已实现（DESIGN 9.3 节）：非运动学 Rigidbody 在步内由软体逐子步推进，步末把组织造成的位移、转动与速度修正加到 PhysX 的结果上（`twoWayCoupling`）。`RigidBodyRestsOnPinnedTissueAndBendsIt`：组织对钢球的力 0.976 N（重量 0.981 N），关闭耦合时钢球穿过组织；S9。限制：Rigidbody 的 Freeze 约束不参与 | 已完成 | 组织能推动带 Rigidbody 的器械 |
| ECO-4 | GPU 后端 | Obi 7 有 Compute Shader 后端 | 没有 | 未开始 | 属于 M4。开始前完成 PERF-9（确定求解方法）与 ENG-7（求解后端接口） |
| ECO-5 | 布料、绳、流体 | 有 | 已冻结（ROADMAP 决策 3）。医疗场景用途：膜与筋膜（布料）、缝线与血管（绳）、出血（流体） | 未开始 | 需先修改 ROADMAP 决策 3 |
| ECO-6 | 粘附与塑性 | 碰撞材质有 stickiness，softbody 有 plasticity | 已实现（DESIGN 9.5 节）：`ApxColliderMaterial.stickiness`、`stickDistance`（接触过的节点被拉回，超过粘附距离后脱离）；`plasticYield`、`plasticCreep`、`plasticRecovery`、`maxPlasticDeformation`（每个体素的塑性伸长，体积不变）。`StickyInstrumentPullsTheSurfaceAfterIt`（粘附时表面上移 1.00 mm，无粘附时 −0.05 mm）、`PressedPlasticBodyKeepsADent`（压痕 11.5 mm，弹性 0.12 mm）；S11 | 已完成 | 湿组织粘附器械；按压后保留残余形变 |
| ECO-7 | 编辑器工具 | blueprint 预计算；用笔刷绘制质量、钉住区域 | blueprint 已实现（DESIGN 9.7 节）：`ApxSoftBodyBlueprint` 资产保存细分网格与体素化结果，检视面板“Bake Blueprint”，匹配时 Start 不细分、不体素化，结果逐位一致（`BlueprintGivesTheSameSimulationWithoutVoxelizing`）。未实现：在编辑器中用笔刷绘制钉住区域与材料（钉住区域仍用 Collider 选取） | 部分完成 | 体素化结果存为资产，Start 不再体素化；在编辑器中绘制钉住区域与材料 |
| ECO-8 | 状态快照与撤销 | 有 | 已实现（DESIGN 9.6 节）：`SoftBodySimulation` / `ApxSoftBody` 的 `CaptureState`、`RestoreState`，含切割拓扑、塑性与抓取状态。`RestoredStateRepeatsTheSameSteps`：切割、恢复后再步进的节点位置与保存后直接步进逐位一致；`RestoredBodyRendersItsTopology`。限制：`ApxSoftBody` 的快照不含碰撞体上一步的位姿（`_previousShapes`，用于子步内插值）与 Rigidbody 的状态；`RestoredStateRepeatsTheSameSteps` 只在仿真层、对静止地面测试，有运动器械或刚体耦合时恢复后能否逐位重复没有测试（与 MED-14 回放相关） | 已完成 | 保存与恢复仿真状态（含切割拓扑），恢复后 `positionsHash` 与保存时一致 |
| ECO-9 | 整体移动与 Transform 变化 | 有 `Teleport`；solver 可在自身局部空间求解，移动 solver 所在对象时其中的物体随之移动 | 节点位置存储在世界空间（DESIGN 第 1 节），静止空间在初始化时取 `lossyScale`（`ApxSoftBody.cs:228`、`257`）。初始化后修改物体 Transform 的位置、旋转、缩放都不影响仿真与渲染：渲染按当前的 `worldToLocalMatrix` 把世界坐标写回局部坐标（`SurfaceMesh.Update`），显示的位置与大小不变；钉住区域为子物体时，只有钉住节点随之移动。以上均没有提示；缩放改变后需调用 `Rebuild()`（切割丢弃）。没有整体平移、旋转物体（含切割后的拓扑）的接口，README 与 DESIGN 第 7 节未说明。节点坐标为单精度世界坐标，远离原点时舍入误差增大：HAP-3 的压痕场景移到 y = 10 m 时，1 mm 深度的夹持力只有压头力的 22%（原点处两者相差 0.1–0.7 mN）。Obi 的物体在 solver 的局部空间求解，solver 移动时按 `worldLinearInertiaScale`、`worldAngularInertiaScale` 传递部分惯性 | 未开始 | 整体移动接口：节点位置、速度、运动学帧与四面体旋转（见 BUG-7）一并变换。EditMode 测试：静止物体整体平移并绕竖直轴旋转 90° 后，无重力 30 帧最大位移小于 0.1 mm；切割后的物体移动后块数不变；初始化后修改缩放时输出警告。在参考空间（物体或一组相互接触的物体共用）的局部坐标中求解，参考空间移动时传递的惯性比例可配置；HAP-3 的压痕场景移到 y = 10 m 后，夹持力与压头力之差小于 1 mN |
| ECO-10 | 事件 | `ObiSolver.OnCollision`、`OnParticleCollision` 输出每步的接触列表；`ObiContactEventDispatcher` 把接触整理为开始、持续、结束三类事件 | 没有事件。切割结果只能由调用方轮询：`ApxScalpel` 的本刀路断棱数、`CountPieces`、`PieceSizes`；碰撞体接触只能逐个查询 `TryGetColliderWrench` | 未开始 | 步末在主线程触发：切割（本步断棱数、刀路编号）、块数变化（新块的编号与实节点数）、碰撞体接触开始与结束（附 HAP-1 的力）、休眠与唤醒。EditMode 测试：box 全切触发一次块数变化事件，两块的节点数与 `PieceSizes` 一致；球压入后抬起，开始与结束事件各一次。MED-13、SUT-8 使用这些事件 |
| ECO-11 | 空间查询 | `ObiSolver.EnqueueSpatialQuery`：球、盒、射线等形状与粒子的相交查询，在求解步中批量执行 | 运行时没有查询接口。射线拾取只在工具程序集中（`ApxTool.PickSurface`），返回最近的节点 | 未开始 | 运行时程序集提供：射线与形变后表面求交，返回交点、法线、块编号与 SUT-1 的材料点；球形区域内的节点查询。EditMode 测试：形变后的物体上射线交点到渲染网格三角形的距离小于 0.1h；`ApxTool.PickSurface` 改用该接口 |
| ECO-12 | 渲染插值 | solver 参数 `interpolation`：显示帧按上两步的结果插值或外推，之后触发 `OnInterpolate` | 在 `ApxSoftBody.FixedUpdate` 中步进，`LateUpdate` 显示最后一步的结果。显示帧率高于物理步频率时（例如 VR 的 72–120 Hz、`fixedDeltaTime` 为 0.02 s），相邻显示帧之间节点位置不变或一次跳过一步；开启插值的 Rigidbody 与组织的显示位置不一致 | 未开始 | 可选的插值模式：表面顶点按上两步的节点位置与 `Time` 中的插值系数计算，不改变仿真结果（`positionsHash` 不变）。EditMode 测试：匀速平移的物体在相邻显示帧之间的显示位移恒定 |
| ECO-13 | 附着约束 | `ObiParticleAttachment`：Static（粒子为运动学，跟随 Transform）与 Dynamic（带柔度的约束，可约束朝向，约束力超过 `breakThreshold` 时断开）；`ObiContactGrabber` 抓取与碰撞体接触的粒子；`ObiStitcher` 连接两个物体的粒子 | 钉住与抓取都把节点设为运动学节点并绑定到运动学帧（`pinColliders`、`BeginGrab`），最多 32 帧（`MaxKinematicFrames`），帧上节点等同固定连接；抓取帧只有平移，每个物体一个（MED-10）；没有带柔度或可断开的连接 | 未开始 | 依赖 SUT-1。附着由一组材料点与一个 Transform 定义，类型为 Static 或 Dynamic；Dynamic 有柔度、可选的朝向约束与断开力阈值，约束力按 HAP-2 的方式输出。钉住与抓取改用附着实现，附着数不受 32 的限制，抓取帧随器械转动。不使用附着时 `positionsHash` 与基线一致；EditMode 测试：Dynamic 附着的支座反力等于自重（误差小于 1%）；拉力超过阈值时附着断开并触发 ECO-10 的事件 |

## 3. 医疗仿真需求

### 3.1 材料

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-1 | 应变硬化（J 形应力-应变曲线） | co-rotational 线弹性，小应变线性化（DESIGN 第 3 节） | 未开始 | 单轴拉伸曲线与所选本构模型的解析解一致 |
| MED-2 | 粘弹性（应力松弛、蠕变） | 没有 | 未开始 | 恒定应变下应力随时间衰减，衰减时间常数可配置 |
| MED-3 | 各向异性（肌纤维方向） | 没有 | 未开始 | 沿纤维方向与垂直方向的刚度比可配置并可测量 |
| MED-4 | 阻尼模型 | 全局速度阻尼（`SolverJobs.cs:163`），刚体运动也被衰减 | 未开始 | 自由下落与匀速平移不受阻尼影响 |
| MED-5 | 多材料与体素材料标签 | 两层：皮肤层与主体（DESIGN 第 3 节）。M5 列出脂肪、肌肉不同材料 | 部分完成 | 每个体素一个材料 ID；从 CT/MRI 分割标签（NIfTI）读取材料 ID |
| MED-6 | 超阈值撕裂 | 没有（M5） | 未开始 | 应变超过阈值时断棱，撕裂沿受力方向扩展 |

### 3.2 切割精度与器械

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| MED-7 | 切口精度 | 分辨率上限 64（`ApxSoftBody.cs:22` 的 Inspector 范围），0.15 m 的物体约 2.3 mm 一个体素；薄于一个体素的结构无法分离（DESIGN 4.3 节）；刀路在体素内弯曲时切口为折线（DESIGN 第 7 节）。2026-10-28 起不属于 M6，排在 PERF-9 与 M4 之后：分辨率 96–128 时 CPU 每步 31–238 ms（DESIGN 6.3 节） | 未开始 | 切口附近局部加密 lattice；弯曲刀路的切口误差小于设定值 |
| MED-8 | 切割阻力与切割力 | 扫掠面与棱相交即断开，组织不先压陷；没有切割力输出 | 未开始 | 切入前组织压陷，切入条件取决于刀锋参数与组织韧性；输出刀刃受力 |
| MED-9 | 器械种类 | 只有线段刀刃（`ApxScalpel`）。缺：剪刀（两刃闭合）、电刀与超声刀（删除体素）、穿刺针（插入不切开）、夹闭器与吻合器。缝合见 3.3 节 | 未开始 | 每种器械一个标准场景截图与 EditMode 测试 |
| MED-10 | 镊子夹持 | 半径内节点设为运动学节点（`BeginGrab`），等同固定连接；每个物体只有一个抓取帧（`SoftBodySimulation.GrabFrame`），帧只有平移（`ApxSoftBody.Simulate` 中的 `float4x4.Translate`）：两把器械不能同时夹持同一物体，夹持处不能扭转 | 未开始 | 两钳口夹持，接触带摩擦，拉力超过阈值时滑脱；同一物体可同时有两处夹持，夹持帧随器械转动 |
| MED-17 | 器械接触的位置与细器械 | 碰撞体接触只作用在 lattice 节点上，所有节点用同一个 `collisionMargin`（`NodeStepJob` 的碰撞体循环），不像物体之间的接触那样按节点自身的静止 SDF 修正：组织表面与器械之间留有不超过约 0.5h 的间隙（DESIGN 第 7 节，可用负的 `collisionMargin` 整体补偿）；比体素小的器械（针尖、剪刀刃、钳口尖）可以从节点之间穿入组织表面而不接触。影响 MED-8、MED-9、MED-10、SUT-2 与力反馈的接触位置 | 未开始 | 碰撞体接触按节点静止 SDF 修正（与物体之间的接触相同），`collisionMargin` 为 0 时静止在 Box 上的物体可见表面与碰撞体的间隙小于 0.1h；半径 0.3h 的胶囊器械压入组织块 2h 深，器械以内没有皮肤网格顶点。该改动会改变落下阶段的结果，基准测试需重新记录对照值 |
| MED-18 | 切下组织块的管理 | 可以读取每个节点的块编号（`SoftBodySimulation.PieceId`，`SoftBodySimulation.cs:140`，拓扑重建时更新）以及块数与各块节点数（`CountPieces`、`PieceSizes`，`SoftBodySimulation.cs:823`、`826`）。不能删除一块（取出的切除标本），也不能把一块分离为单独的 `ApxSoftBody`：切下的块继续参与求解、接触与渲染，每步耗时不减少；休眠按整个物体的节点最大速度判断（`ApxSoftBody.cs:337`），一块运动时其余块都不休眠。电刀、超声刀删除体素（MED-9）需要同样的删除机制 | 未开始 | 按块查询节点、体素副本与包围盒；删除指定的块后，其节点、体素副本与渲染三角形移除，其余部分继续步进且结果有限。EditMode 测试：box 全切后删除一块，剩余实节点数与体素副本数等于另一块的数目；删除后 `RestoreState` 可恢复到删除前 |

### 3.3 缝合

缝合由三部分组成：缝针穿刺组织、缝线在穿刺通道内滑动、收紧缝线使切口两侧对合。SUT-1 是其他条目的基础；
SUT-5 不模拟缝线，可以在缝针与缝线之前实现，用于验证切口闭合与组织张力。

| 编号 | 需求 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| SUT-1 | 材料点锚定 | 抓取与钉住通过节点列表绑定到运动学帧（`Attach`，`SoftBodySimulation.cs:714`；最多 32 帧，`MaxKinematicFrames`，`SoftBodySimulation.cs:71`）；没有嵌入在体素副本内的材料点 | 未开始 | 材料点由（体素副本，局部坐标）定义，位置按三线性插值计算；切割后随副本继承；可作为穿刺点、缝线锚点、力反馈接触点使用 |
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
| HAP-1 | 碰撞体受力：器械（Collider）受到的力、力矩、接触中心 | 已实现（DESIGN 9.2 节）：节点修正的冲量按 256 个节点一块累加、步末按固定顺序求和；`ApxSoftBody.TryGetColliderWrench`、`ColliderWrenches`、`ApxHaptics.TrySumColliderWrench`、`SoftBodySimulation.TryGetShapeWrench`。力矩参考点为碰撞体形状的参考点（球、盒、距离场为中心，胶囊为第一个端点），与下方草案中的 Transform 位置不同。`ColliderForceEqualsTheWeightOfARestingBody`：−9.790 N，重量 9.810 N（偏差 0.2%） | 已完成 | 节点 i 对碰撞体 s 的位置修正 Δx（法向投影加摩擦）换算为力 `m_i Δx / h²`（h 为子步长），对节点求和取反；力矩相对碰撞体形状的参考点（球、盒、距离场为中心，胶囊为第一个端点）。静止物体放在 Box 碰撞体上，碰撞体受力等于总重量（误差 < 2%） |
| HAP-2 | 运动学帧受力：抓取帧、钉住区域的支座反力 | 已实现（DESIGN 9.2 节）：`SolveCellsJob` 累加约束对运动学节点的 `∇C λ`（按（切片, 帧）求部分和，步末按切片序号求和，不使用原子操作），一步的力为约束力的子步平均加上帧上节点自身的重量；`SoftBodySimulation.TryGetFrameWrench`、`ApxSoftBody.TryGetGrabWrench`、`TryGetPinWrench`（草案中的 `GrabWrench`、`GetPinWrench` 改为与 HAP-1 一致的 Try 形式）。力矩相对帧在步末的原点（抓取目标、钉住碰撞体位置）。`GrabForceEqualsTheWeightOfALiftedBody`：−9.779 N，重量 9.810 N（偏差 0.3%；被抓取的 39 个节点重 0.383 N，不计入时偏差 4.2%）；`PinForcesCarryTheWeightOfAPinnedBody`：两端合计 −1.436 N，重量 1.440 N（偏差 0.3%）。步末弹性力 `−∂E/∂x` 的方法（试验，未采用）：同一抓取场景在分辨率 8、16、24 时偏差 +0.8%、+5.8%、+24.4%，约束力均为 −0.3%。不计入：被抓取组织随帧加速的惯性力；块间与物体间接触对运动学节点的反作用。没有运动学节点时基准测试 16 组的 `positionsHash` 不变 | 已完成 | 两种候选方法：在 `SolveCellsJob` 中累加约束对运动学节点的 `λ∇C / h²`；或在步末对运动学节点计算弹性力 `−∂E/∂x`。按下面两项测试选择：用抓取提起整个物体静止后，抓取受力等于总重量（误差 < 2%）；两端钉住的物体静止时，钉住反力之和等于总重量 |
| HAP-3 | 力的物理校验测试 | 已实现（DESIGN 9.2 节，`InteractionTests`）：自重 3 项（碰撞体偏差 0.2%，抓取、钉住 0.3%）；`InstrumentAndPinForcesBalanceTheWeight`：两端钉住的组织条被球压入 5 mm，静止后球与两端受力之和与重量相差 0.28%，对原点的力矩之和 (−2.687, 0.006, 0.214) × 10⁻³ N·m，重量的力矩 (−2.699, 0, 0.174) × 10⁻³ N·m；`IndenterForceGrowsWithDepthAndMatchesHertz`：E = 3 kPa、ν = 0.45、体素 2.5 mm、64 子步，力随深度单调增加，与 Hertz 解之比 0.854、0.989、1.021、1.048（0.5、1、2、3 mm）；`SameInputsGiveBitIdenticalForces`：两次运行的碰撞体与帧受力逐位一致。另外的结果：每个子步一次迭代使 lattice 的实际刚度随 μΔt²/(ρh²) 降低，默认参数（30 kPa、ν = 0.45、16 子步）下压痕力为 Hertz 解的 20–32%（体素 5 mm）、9–15%（体素 2.5 mm），定量使用前需标定（CFG-10）或改进求解（BUG-3、PERF-9）；节点坐标的单精度舍入影响 mN 级的力：64 子步时压头力与夹持力相差 0.1–0.7 mN，同一场景移到 y = 10 m 时 1 mm 深度的夹持力只有压头力的 22% | 已完成 | EditMode 测试：自重（HAP-1、HAP-2 中的两项）；力平衡（器械力 + 钉住反力 + 重力 = 0，物体静止时）；球形压头压入组织块，力随深度单调增加，小深度下与 Hertz 接触解比较（容差 30%，与 `SelfWeightCompressionMatchesYoungsModulus` 同级）；相同输入时力逐位一致 |
| HAP-4 | 局部线性力模型：供 1 kHz 线程在两次物理步之间计算力 | 已实现（DESIGN 9.2 节）：每步为每个接触中的碰撞体发布 `ApxForceModel`（`ApxSoftBody.TryGetColliderForceModel`、`ApxHaptics.TrySumColliderForceModel`、`ApxForceModelEstimator`）。x0 为碰撞体参考点在力的各采样时刻的平均位置，F0 为该步的平均力；K = −k N，N 为节点 job 累加的接触法向张量（`ApxWrench.ContactNormals`，单位迹），k 为沿运动方向的有限差分 −s·y / (sᵀ N s)，K 对称半负定。`Evaluate` 只推不拉，可设 `MaxForce`。选择依据（试验，默认材料，压入 12 mm/s 时预测下一步的最大误差）：只用上一步的力 28.6–33.8%，名义刚度的 Σ k n nᵀ 248–940%（接触刚度与接触半径而非节点数成比例），对称割线更新与采用的方法均为 5.8–7.8%；后两者在压入方向上相同，采用法向张量是因为 K 天然对称半负定、未运动过的方向也有接触几何。`ForceModelPredictsTheNextStepOfAnIndentation`：压入 1–6 mm 的 26 步最大误差 6.0%（只用上一步的力 35.6%）。限制：抓取帧与钉住区域没有力模型；不含力矩与摩擦黏着的切向刚度；接触开始后 1–2 步刚度偏小（压入 30 mm/s 时 1 mm 处误差 67–81%）；体素 5 mm 时新一层节点进入接触的那一步误差较大（30 mm/s 时最大 20.6%）。基准测试 16 组的 `positionsHash` 不变 | 已完成 | 每步发布 `F(x) = F0 + K (x − x0)`，K 为 3×3 刚度（N/m），K = −k N（接触法向张量乘沿运动方向的差分刚度，选择依据见 DESIGN 9.2 节）。验收：压头匀速压入时，用模型预测下一步的力，相对误差 < 10%。测试条件：2 cm 球以 12 mm/s 压入底面夹持的块，体素 2.5 mm，压入 1 mm 以后（`ForceModelPredictsTheNextStepOfAnIndentation`，最大 6.0%） |
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
    public float3 Origin;           // 力矩参考点：碰撞体形状的参考点（HAP-1），或运动学帧原点（HAP-2）
    public float3 ContactCentroid;  // 按力大小加权的接触点
    public int ContactNodes;
}

public partial class ApxSoftBody
{
    // HAP-1：上一步中该碰撞体受到的力；本步未接触时返回 false
    public bool TryGetColliderWrench(Collider collider, out ApxWrench wrench);
    // HAP-2：抓取帧与钉住区域（pinColliders 的下标）受到的力；未抓取、区域没有节点时返回 false
    public bool TryGetGrabWrench(out ApxWrench wrench);
    public bool TryGetPinWrench(int pinIndex, out ApxWrench wrench);
    // HAP-4：上一步中该碰撞体的局部力模型；本步未接触时返回 false
    public bool TryGetColliderForceModel(Collider collider, out ApxForceModel model);
}

public static class ApxHaptics
{
    // 所有 ApxSoftBody 对同一碰撞体的力之和；没有物体接触该碰撞体时返回 false
    public static bool TrySumColliderWrench(Collider collider, out ApxWrench sum);
    // 所有 ApxSoftBody 对同一碰撞体的力模型之和（移到第一个模型的位置）
    public static bool TrySumColliderForceModel(Collider collider, out ApxForceModel sum);
}

// HAP-4：局部线性力模型，每个物理步发布一次
public struct ApxForceModel
{
    public double Time;        // 仿真时间，s
    public float3 Position0;   // 器械参考点在该步力的各采样时刻的平均位置
    public float3 Force0;      // 该步的平均力
    public float3x3 Stiffness; // dF/dx，N/m；−k N，对称半负定
    public float MaxForce;     // 力上限，N；0 为不限制
    public float3 Evaluate(float3 position);   // 与 Force0 反向时为 0
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
| MED-13 | 评分事件与指标 | 没有。需要：切口长度、深度、位置，组织损伤量，超力事件，缝合指标（SUT-8）。事件来源见 ECO-10 | 未开始 | 事件 API 与一个标准场景的评分输出 |
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
| CFG-6 | 只读派生量 | 已有：`CellSize`、`LastStepMilliseconds`、`LastTopologyMilliseconds`、`LastInitializeTimings`、`TotalMass`、`SimulatedTime`（HAP-4）。缺：子步 dt、稳定性数 μΔt²/(ρh²)（DESIGN 第 3 节） | 部分完成 | 派生量可通过 CFG-5 的接口读取 |
| CFG-7 | 硬编码常量改为参数 | `SleepSteps`、`SleepSpeed`（`ApxSoftBody.cs:108`、`110`）；接触搜索半径 1.8h、最小距离 0.8h（`SoftBodySimulation.cs:514`、`533`、`534`）；扫掠延伸 0.5h / 0.1h（`SoftBodySimulation.cs:664`）；`ContactInterval`（`SoftBodySimulation.cs:74`）；`MaxKinematicFrames`（`SoftBodySimulation.cs:71`），钉住区域因此最多 31 个（帧 0 为抓取），`pinColliders` 超出的部分在初始化与步进中直接跳过（`ApxSoftBody.cs:265`、`324`），没有提示；碰撞体查询数组长度 64（`ApxSoftBody._overlap`，周围碰撞体超过 64 个时 `OverlapBox` 截断，没有提示）；物体之间的接触：候选副本数 12（`BodyContacts.MaxCandidates`）、每子步推出上限 0.25h、参与接触的节点静止 SDF 下限 −0.25h、梯度长度下限 0.5（DESIGN 9.4 节）；同一块的折叠接触：候选副本数 8（`Contacts.MaxFoldCells`）、静止邻域 4 × 4 × 4 个体素（`Contacts.FoldRestCells`）、参与节点的静止 SDF 下限 −0.75h、候选筛选的余量 0.5h 与路径上限 3h、松弛系数 0.5、每子步推出上限 1h（DESIGN 3.1 节）；节点质量下限为体素质量的 1/16（`TopologyJob.MinNodeMass`）；MeshCollider 运行时烘焙的距离场分辨率（`ApxColliderMaterial.DefaultFieldResolution`） | 未开始 | 常量带 CFG-1 元数据，默认值不变时 `positionsHash` 不变；碰撞体数超过查询上限、钉住区域数超过帧上限时输出警告 |
| CFG-8 | unity-mcp 自定义工具：`apx_get_params`、`apx_set_params`、`apx_run_scenario`、`apx_benchmark` | 基准测试可通过 MCP 的代码执行调用 `ApxBenchmark.ScheduleKeepingScenes`；没有自定义工具 | 未开始 | 四个工具可用；长时间任务异步执行并可查询结果 |
| CFG-9 | 文本描述的实验场景（刀路、参数、期望指标），batchmode 运行并输出 JSON | 基准测试输出 JSON，场景固定在代码中；标准场景截图工具（S1–S6、交互场景 S8–S11）只输出 PNG 与日志 | 部分完成 | 场景文件定义刀路与期望指标；S1–S6、S8–S11 输出 JSON 指标 |
| CFG-10 | 闭环调参示例 | 没有。依赖的 HAP-1、HAP-3 已完成；HAP-3 的压痕结果显示默认参数下 lattice 比名义杨氏模量软（DESIGN 9.2 节），调参以仿真得到的力—位移曲线为准 | 未开始 | 调整材料参数，使压痕的力-位移曲线与给定文献数据的误差小于设定值 |

## 5. 性能与基准测试

| 编号 | 内容 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| PERF-1 | 基准测试框架 | `AdaptorPhysX/Run Benchmarks`、`ApxBenchmark.RunBatch`，网格 box、liver、bunny、dragon，分辨率 16/32/48/64，JSON 输出（DESIGN 第 8 节），结果见 DESIGN 6.1 节 | 已完成 | — |
| PERF-2 | 基准矩阵的其余维度 | 未覆盖：分辨率 96/128（`resolution` 可由代码设置，Inspector 范围为 4–64）；子步 8/32（`-apxSubsteps` 已支持，未运行）；抓取状态；Armadillo、Thingi10K、真实器官分割网格。2026-10-28：工程内的模型资产可作为基准网格（网格名为 `Assets/...` 路径，顶点取到模型根节点的空间），已运行 3 个 FBX 测试模型（DESIGN 6.2 节）；静止 lattice 中间层的 z 离开落地后倾倒的物体时，刀面改取物体包围盒中心。分辨率 96、128 已运行（2026-10-28，DESIGN 6.3 节，box、Fertility、龙的 FBX）：每步 31–238 ms，切割帧的仿真拓扑重建 62–660 ms，落下后体积比 0.895–0.945，切割帧渲染 37–100 ms；Fertility 128 全切未分开（BUG-14）。未覆盖：子步 8/32、抓取状态、Armadillo、Thingi10K、器官分割网格 | 部分完成 | 上述维度各有一组结果 |
| PERF-3 | 指标缺口 | Native 内存只有 Unity 总分配量的差值（编辑器中为近似值）；托管分配为 Mono 堆增长量，期间发生 GC 时记为 -1；首次 JIT 耗时在预热中，未单独记录 | 部分完成 | 记录仿真与表面网格的 Native 内存；首次切割帧耗时单独记录 |
| PERF-4 | batchmode、Player（IL2CPP）与 ARM 数据 | 6.1 节数据来自交互式编辑器（Debug、Jobs Debugger 开启）。batchmode 已运行（`ApxBenchmark.RunBatch`，2026-10-27，16 组，结果文件在 `Benchmarks/Results/`，不提交）；Windows Player（IL2CPP）与 Quest 未测 | 部分完成 | batchmode 运行一次；Windows Player（IL2CPP、Release）与 Quest 各一组结果 |
| PERF-5 | 仿真拓扑局部重建 | 每个切割帧全量重建：1428 个体素 1.2 ms，77220 个体素 51 ms（6.1 节）；分辨率 96–128（5.9 万–57.7 万个体素）62–660 ms（6.3 节） | 未开始 | 77220 个体素时切割帧扫掠小于 5 ms；属于 M3，同时是 M6 的剩余条目 |
| PERF-6 | 表面更新与渲染拓扑重建 | 龙（43.5 万顶点）每帧表面更新 8–11 ms，切割帧表面重建 p95 200–460 ms；box、liver、bunny 分辨率 64 时 p95 约 160 ms。真实网格摸底（DESIGN 6.2 节，2026-10-28）：Fertility（24 万顶点）切割帧重建 p50 / p95 在分辨率 16 时为 556 / 1144 ms、64 时为 90 / 198 ms，龙的 FBX 16 时为 1222 / 2025 ms，交互中刀每经过一帧就卡顿这么久。分阶段耗时（`SurfaceMesh.LastTimings`）：高面数网格以托管代码的三角形拆分为主（龙 16 为 400 ms，网格拓扑上传 42 ms），粗网格、高分辨率以切面几何为主（box 64 为 36.5 ms）；龙每帧顶点写入 6–10 ms、蒙皮 1–3 ms。2026-10-28 已改（DESIGN 第 5 节）：顶点分两个流，位置与法线每次更新一次写入（`SetVertexBufferData`），包围盒取 lattice 节点的包围盒，索引一次写入（`SetIndexBufferData`）；触及切开体素的皮肤三角形在 Burst 中并行拆分（`SplitTrianglesJob`、`MergeSplitsJob`），去掉托管的拆分缓存，容量不足的三角形仍用托管代码（两者结果相同，`SplitJobMatchesTheManagedSplit`）。同一时段前后对比（DESIGN 6.2 节）：切割帧重建 p50 / p95 龙的 FBX 16 由 2231 / 3941 ms 降至 85 / 164 ms，Fertility 16 由 1176 / 2857 降至 77 / 165 ms，dragon 64 由 269 / 483 降至 77 / 126 ms，box 64 由 106 / 383 降至 72 / 216 ms；每帧表面更新龙由 13–15 ms 降至 3.6–5.2 ms。剩余：网格拓扑上传（高面数网格约 40 ms）、切面几何（托管代码，复制仿真数组并遍历全部体素，box 64 为 57 ms）、拆分 10–40 ms；未达到验收的 16 ms，顶点也未在 GPU 上写入。2026-10-28 第二部分（DESIGN 第 5 节）：切面、裂缝面、拟合平面、阻挡三角形与可见副本移到 Burst（`CutGeometryJob`、`MergeCutGeometryJob`、`VisibleCopiesJob`），托管快照只在需要托管拆分时复制；源顶点的单元坐标与静止法线常驻，重建时不再复制；表面更新的计时分出网格上传（`Upload`）。同一批 batchmode 运行的前后对比（相隔约 20 分钟，静止步耗时相近）：切割帧重建 p50 / p95 box 64 由 63.9 / 184.9 ms 降至 22.6 / 49.3 ms（切面几何 51.0 → 3.3 ms），bunny 64 由 66.2 / 212.9 降至 16.1 / 29.8 ms，liver 64 由 41.3 / 179.9 降至 14.9 / 22.5 ms。高面数网格剩下网格上传（Mesh API，龙约 16–25 ms：每次重建整体写入约 10 MB 索引与 9 MB 静态顶点）与拆分（10–45 ms；这段时间另一个编辑器同时在运行，同一代码的拆分耗时在 30–45 ms 之间波动）。去掉 `Mesh.Clear()`、只调整缓冲大小的对照没有稳定收益，未采用。测试 `SplitJobMatchesTheManagedSplit` 改为 `RebuildJobsMatchTheManagedRebuild`，同时比较切面几何 | 部分完成 | 切割帧表面重建 p95 小于 16 ms；顶点在 GPU 上写入（M4） |
| PERF-7 | 初始化耗时 | box 64 的 4.78 s 中表面细分 3.67 s（`MeshUtil.Subdivide`，托管 List）；龙 64 的 3.93 s 中体素化 2.43 s（`InsideParityJob` 每行遍历全部三角形）。使用 ECO-7 的 blueprint 时两项均跳过；不使用 blueprint 时仍为上述耗时。真实网格（DESIGN 6.2 节）：扫描版兔子 64 的 4.42 s 中准备（含细分）3.20 s；分辨率 128 时体素化约 5 s。2026-10-28 已改（DESIGN 第 2 节）：射线奇偶判定按 (y, z) 行把三角形分桶，近表面距离按 z 层并行，表面细分移到 Burst；节点位置哈希与体素数不变（基准测试 16 组与基线一致，3 个 FBX 模型 12 组前后相同）。同一批 batchmode 运行的前后对比（不使用 blueprint）：box 64 初始化 6.44 → 2.29 s（细分 5.05 → 0.76 s），dragon 64 7.48 → 1.70 s（体素化 4.52 → 0.59 s），Fertility 64 3.73 → 1.38 s，扫描版兔子 64 6.97 → 2.64 s（细分 4.90 → 0.59 s；缠绕数判定未改，体素化仍为 1.7 s）。剩余：体素化后的托管循环（填充率、深度、法线，逐体素）、缠绕数判定、`SurfaceMesh` 构造时逐顶点查找所在体素 | 部分完成 | 两项各降低一个数量级；或由 ECO-7 的 blueprint 缓存替代 |
| PERF-8 | 自适应 lattice | 分辨率上限 64，lattice 均匀 | 未开始 | 与 MED-7 一起验收，排在 PERF-9 与 M4 之后 |
| PERF-9 | 减少子步数的求解方法 | 16 子步、每子步 1 次迭代，λ 每子步从 0 开始。默认参数下 lattice 的实际刚度低于名义杨氏模量：压痕力为 Hertz 解的 9–32%（HAP-3），落下后体积比 box 64 为 0.972、box 128 为 0.898（BUG-3），有摩擦时漂移 1–2 mm（BUG-2）。候选：子步之间沿用 λ（warm start）、Chebyshev 加速、Vertex Block Descent | 未开始 | 评估报告：相同耗时下比较各方法的 Hertz 压痕力（HAP-3 的测试）、落下后体积比（BUG-3）与摩擦漂移（BUG-2）。结论决定 M4（ECO-4、ENG-7）的求解方法，以及 CFG-4、CFG-10、MED-1、MED-6 的开始时间 |
| PERF-10 | 开放网格的内外判定 | 已实现（DESIGN 第 2 节）：网格有边界棱时（BUG-10 的统计）格点内外改用广义缠绕数判定（Barill 等 2018 的一阶近似：三角形按重心 Morton 码排序后二分成树，叶节点 8 个三角形，距离大于节点半径 2 倍时用偶极子近似，否则精确计算立体角），缠绕数绝对值大于 1/2 为内部，孔在 lattice 中封闭，渲染表面保留孔；闭合网格仍用射线奇偶，结果不变。扫描版兔子 FBX（223 条边界棱）分辨率 64：两种判定在底部 34 行、817 个格点上不同（奇偶判定在经过孔的行上把 491 个外部格点判为内部、326 个内部格点判为外部），缠绕数判定没有这类行错误；体素化 761 ms（奇偶 675 ms），分辨率 128 时 5.7 s（主要为单线程的近表面距离，见 PERF-7）。限制：SDF 的大小仍为到最近三角形的距离，孔处为到孔边缘的距离 | 已完成 | `OpenMeshIsVoxelizedLikeTheClosedMesh`：去掉 −x 面的盒子（16 条边界棱）按缠绕数判定的格点内外与闭合盒子完全相同、体素相同，奇偶判定有错误的格点；闭合盒子按缠绕数判定与奇偶判定相同。扫描版兔子的判定差异只在孔所在的格点行；基准测试 16 组与基线一致 |

## 6. 发布与工程

| 编号 | 内容 | 现状 | 状态 | 验收 |
|---|---|---|---|---|
| ENG-1 | 以 UPM 包发布 | 插件在 `Assets/AdaptorPhysX`，没有 `package.json`、版本号与 CHANGELOG；示例在 `Assets/Samples`，不是包内的 `Samples~` | 未开始 | 另一个空工程用 git URL（`?path=`）引用包后编译通过，示例可从 Package Manager 导入，包内 EditMode 测试可运行 |
| ENG-2 | 自动运行测试 | EditMode 与 PlayMode 测试只在本机 batchmode 或编辑器中手动运行（README）；没有 CI | 未开始 | 推送后自动运行 EditMode 与 PlayMode 测试并保存结果文件；运行环境的配置（含 Unity 许可证）写入 README |
| ENG-6 | Unity 6 支持 | 工程为 Unity 2022.3.54f1 与内置渲染管线。Obi 7.2 的最低版本为 Unity 6000.0.82f1 | 未开始 | Unity 6 LTS 打开工程后编译通过，EditMode 与 PlayMode 测试通过，标准场景 S1–S12 截图正常；Burst 版本变化使 `positionsHash` 改变时重写基线并在提交中说明。迁移时机待定（ROADMAP“待决定”第 1 项），可与 MED-15 一起进行 |
| ENG-7 | 求解后端接口 | 求解直接写在 `SoftBodySimulation` 与各个 job 中，`ApxSoftBody`、`SurfaceMesh` 直接读写其中的 NativeArray。Obi 7 先定义 `IObiBackend`、`ISolverImpl`，Burst、Compute 与空后端各实现一套 | 未开始 | 定义求解后端接口（步进、切割、接触数据、渲染与力的读取），现有 Burst 实现放在接口之后，`positionsHash` 与基线一致。ECO-4 的前提 |
| ENG-3 | 基准测试基线 | 已实现（DESIGN 第 8 节）：`Benchmarks/baseline.json` 入库，记录每组 4 个阶段的 `positionsHash`、全切块数、断棱数、最大的 4 块节点数，以及环境（Unity 版本、CPU、操作系统、托管代码优化、Burst）、设置与网格的顶点数、三角形数（`BenchmarkBaseline`）。基准测试结束时比较两者共有的组，列出不一致的字段；设置或网格不同时不比较；环境不同时给出警告（哈希只在同一机器、同一构建下可比）；`-apxFailOnBaselineDifference` 时不一致则退出码为 1。基线来自 `1f9bcaa`（运行时代码与 `b891564` 相同），在交互式编辑器中运行，16 组与 BUG-12 修复后在工程副本上用 batchmode 运行的结果逐项相同，说明两种运行方式的哈希可比 | 已完成 | 基线文件（例如 `Benchmarks/baseline.json`：每组 4 个阶段的 `positionsHash`、块数、断棱数，以及机器与构建信息）入库；基准测试运行后与基线比较，列出不一致的组 |
| ENG-4 | 标准场景截图与里程碑验收 | 此前 `docs/images` 中 S1–S6 的截图来自 `1ef1a09`（2026-10-26），早于 BUG-1、BUG-4、BUG-6、BUG-7；S8–S11 来自 `1e35fa0`，早于 BUG-8 的同步子步（`2aec0fe`）。2026-10-27 按 `7389533` 在 batchmode 中重新截取 S1–S6、S8–S11 并更新 `docs/images`：S3 原截图中各块仍贴在一起，看不出切口，截图工具在 S3 之后拉开后侧三块，新增 S3b（5 块分离，切面完整）。与此前的截图相比，S1–S6 除背景中组织块的位置外差别在几千像素以内，S8 中肝叶位置略有变化，S10、S11 逐像素相同；ROADMAP“当前结果”各项仍成立（S8 补充两个物体之后都进入休眠）。S3b 中皮肤边缘与切面交界处有 1–2 像素的台阶（DESIGN 第 7 节，每个体素一个拟合平面）。之后 BUG-11、BUG-12 更新了有切口变化的截图。M1、M2 的人工验收（ROADMAP 决策 5）：2026-10-28 用户试用后确认基本形状上的柔体与交互作为第一个版本通过，已记入 ROADMAP；真实网格不在此次验收范围内（M6） | 已完成 | 用当前提交重新截取 S1–S6、S8–S11，更新 `docs/images` 与 ROADMAP“当前结果”列；M1、M2 的人工验收结果记入 ROADMAP |
| ENG-5 | 交互工具与调试显示 | 已实现（DESIGN 第 10 节，程序集 `AdaptorPhysX.Tools`，参照 Obi 的粒子渲染组件与 blueprint 编辑器的显示模式）：相机绕中心点旋转、平移、向指针缩放、框选与预设视角（`ApxCameraRig`）；手术刀的航向、侧倾、前倾与竖直深度可调，按下后移动 3 mm 才起刀、航向取刀路方向，可在组织旁起刀，Shift 沿航向直线切割（`ApxScalpelTool`，`ApxBladePose`）；镊子滚轮提起、Shift 松开后保持（`ApxForcepsTool`）；测量点随节点移动（`ApxMeasureTool`）；调试图层：网格、包围盒、lattice、节点、断棱与切口张开量、受力箭头与数值、刀刃与扫掠面，节点按块、运动学帧或速度着色，X-ray 与隐藏表面（`ApxDebugView`）；暂停与单步；面板显示工具状态、按键与各物体统计。两个演示场景改用这些工具（删除 `SurgeryDemoController`），标准场景截图与修改前逐像素相同，截图工具新增 D1–D3。EditMode 测试 6 个（`ToolTests`）。限制：刀的参考方向为世界竖直轴，不按表面法线；镊子只有平移（MED-10）；只支持旧输入系统；单步为一帧的固定步；人工试用中发现手动切割的切口呈锯齿（BUG-11，已修复：固定指针平面、稳定绳、侧别断棱的切口点） | 已完成 | 相机旋转、平移、缩放与预设视角；刀刃朝向可调，切割按该朝向进行；调试图层在 Game 与 Scene 视图中可见；EditMode 测试覆盖刀刃几何、相机数学、三种工具与 lattice、断棱图层 |

## 7. 建议顺序

| 顺序 | 条目 | 状态 |
|---|---|---|
| 1 | 基准测试框架（PERF-1） | 已完成 |
| 2 | 切割与接触缺陷（BUG-1、BUG-2、BUG-4、BUG-6），之后 BUG-3 | 部分完成（BUG-1、BUG-4、BUG-5、BUG-6、BUG-7、BUG-11、BUG-12、BUG-15、BUG-17 已完成；BUG-2、BUG-3 原因已确认，修复需改动求解器；BUG-9 原因已确认；BUG-16 部分完成） |
| 3 | 参数元数据、生效时机、MCP 工具（CFG-1、CFG-2、CFG-5、CFG-6、CFG-8；同时处理 BUG-5） | 部分完成（CFG-2 已完成） |
| 4 | 跨物体碰撞、MeshCollider 与 SDF 器械、局部重建（ECO-1、ECO-2、PERF-5、PERF-6） | 部分完成（ECO-1、ECO-2 已完成，摩擦地面上的推动见 BUG-8；同时完成 ECO-3、ECO-6、ECO-8，ECO-7 部分完成） |
| 5 | 力反馈计算接口（HAP-1 至 HAP-7，按编号顺序） | 部分完成（HAP-1 至 HAP-4 已完成） |
| 6 | 体素材料标签 → 非线性与粘弹性材料 → 撕裂（MED-5、MED-1、MED-2、MED-6） | 未开始 |
| 7 | 缝合：SUT-1、SUT-5、SUT-9 并入第 9 项；SUT-2 至 SUT-4、SUT-6 依赖穿刺（MED-9）与缝线（ECO-5） | 未开始 |
| 8 | 真实网格（ROADMAP M6）：ENG-3、BUG-10 → 用高面数测试模型摸底 → PERF-10、PERF-7、PERF-6 → PERF-5、BUG-13 剩余部分（bunny 64 落地后耳朵持续运动）。2026-10-28 收窄为分辨率不超过 64；PERF-2 的高分辨率数据已记录，MED-7、PERF-8 移到 PERF-9 与 M4 之后 | 进行中（ENG-3、BUG-10、PERF-10 已完成；PERF-6、PERF-7、BUG-13 部分完成；PERF-5 未开始） |
| 9 | 组织操作闭环（ROADMAP M7）：SUT-1 → ECO-13 → MED-17 → MED-10 → SUT-5、SUT-9 → ECO-10、MED-13 评分示例、MED-14；ECO-11 在需要从表面选取材料点时实现 | 未开始 |
| 10 | 求解方法评估（PERF-9，与 BUG-3、BUG-2 一起），与第 9 项并行 | 未开始 |
| 11 | 运行时与平台：ECO-9（局部参考空间）、ECO-12（渲染插值）、ENG-6（Unity 6）与 MED-15（URP）、ENG-7（求解后端接口，M4 之前） | 未开始 |

第 2 项排在参数化之前：BUG-1 与 BUG-2 影响切割结果与基准测试中的“全切结果”列，后续改动需要以此为对照。
2026-10-28 起第 8 项优先：基本形状上的柔体与交互作为第一个版本通过人工验收（ROADMAP M1、M2），下一阶段的目标为真实网格。
第 8 项完成后第 9 项为主线，第 10 项并行。第 9 项中只有 MED-17 改变单物体基准的结果（落下阶段，需重写基线），其余条目在不使用附着时 `positionsHash` 不变；第 10 项的对比在 MED-17 之后进行时以重写后的基线为准。
暂缓：M4（ECO-4，等第 10 项的结论与 ENG-7）；MED-1 至 MED-3、MED-6、CFG-4、CFG-10（等第 10 项的结论）；HAP-5 至 HAP-7（没有设备）；ECO-5。
ENG-6 与 MED-15 的时机、缝线的实现方式见 ROADMAP“待决定”。

## 变更记录

| 日期 | 提交 | 内容 |
|---|---|---|
| 2026-10-26 | `17fde10` | 基准测试框架（PERF-1）；基准测试与代码检查发现 BUG-1 至 BUG-5 |
| 2026-10-26 | `7ee1147` | 建立本文件 |
| 2026-10-26 | `2bc9975` | 新增 3.3 节缝合（SUT-1 至 SUT-9）；力反馈改为计算接口范围（3.4 节，HAP-1 至 HAP-7 与接口草案），替代原 MED-11、MED-12 |
| 2026-10-26 | `8eda481` | BUG-4 已完成：端点在刀面上的棱按有符号距离归到刀面正侧；新增 EditMode 测试；基准测试 `positionsHash` 与修复前一致 |
| 2026-10-26 | `db3ad63` | 演示：刀刃深度从刀下组织表面算起，拖动时随表面高度变化。新增 BUG-6；BUG-1 补充证据 |
| 2026-10-26 | `44aa786` | BUG-2 原因已确认，记录求解顺序与镜像剖分实验（未采用）；新增并完成 BUG-7（旋转放置的物体变形）；BUG-1 补充证据 |
| 2026-10-27 | `c6e5da1` | BUG-1 已完成（刀路内节点侧别）；BUG-5、CFG-2 已完成（参数下一步生效、`Rebuild()`）；BUG-3 原因已确认；BUG-2 补充对称 Gauss-Seidel 实验 |
| 2026-10-27 | `1e35fa0` | 与 Obi 的差距：ECO-1、ECO-2、ECO-3、ECO-6、ECO-8、HAP-1 已完成，ECO-7 部分完成（blueprint），PERF-7 部分完成；CFG-6 增加 `TotalMass`。新增 EditMode 测试 10 个（`InteractionTests`）、交互演示场景与标准场景 S8–S11。默认设置下基准测试 16 组的 `positionsHash`、块数与断棱数与 `c6e5da1` 一致 |
| 2026-10-27 | `8133858` | BUG-6 已完成：断开刀尖、刀跟外侧的表面外棱；皮肤顶点的副本按副本边界断棱判断侧别，同一切口的平面按三角形处的距离去重；新增 EditMode 测试；基准测试 `positionsHash` 与不启用规则时一致 |
| 2026-10-27 | `2aec0fe` | ECO-2 改为部分完成：MeshCollider 与 `ApxDistanceField` 资产路径没有 EditMode 测试，验收改为对应测试；修正过期的文件行号（以 `20c1e5e` 为准） |
| 2026-10-27 | `2aec0fe` | ECO-1 改为部分完成：验收中的相向挤压没有测试（`StackedBodiesRestOnEachOther` 只覆盖静止堆叠），验收改为分列已覆盖与未覆盖两部分 |
| 2026-10-27 | `2aec0fe` | ECO-2 已完成：新增 `MeshColliderAndFieldAssetFloorsHoldTheBodyLikeABoxCollider`（MeshCollider 与 `ApxDistanceField` 资产地面，最低节点高度与 Box 碰撞体之差 0.000 mm）。新增 ECO-1 相向挤压测试 `PinnedBodyPushesAnotherWithoutPassingThrough`，未通过，标为 `[Ignore]`；新增 BUG-8（持续挤压时软体之间相互穿透）。运行时代码未改动 |
| 2026-10-27 | `2aec0fe` | BUG-8 原因已确认：步内对方副本不更新、按份额推出与每子步 0.25h 的上限，使持续载荷下的残余重叠随载荷增大；越过 SDF 精确带或位于对方 SDF 中轴（梯度为 0）的节点不再被检测。`PinnedBodyPushesAnotherWithoutPassingThrough` 的穿入深度改为按物体间接触的定义计算（原几何深度在角节点上偏大）。运行时代码未改动 |
| 2026-10-27 | `2aec0fe` | BUG-8 部分完成：相互接触的物体同步推进子步（`SubstepGroup`，每个子步求解后刷新对方副本），本步不推进的对方作为静止障碍；物体间接触另用补全深处的格点 SDF（`ContactSdfJob`）。新增 `SqueezedBodiesKeepTheirContactUnderASustainedLoad`、`PinnedBodyPushesAnotherAlongAFrictionlessFloor`；地面摩擦下的推动仍穿透（被推物体倾倒，受力集中在推动体底边），ECO-1 验收测试保持 `[Ignore]` |
| 2026-10-27 | `8180ec3` | BUG-8：试过反作用传递与按自身移动放宽推出上限，均使静态挤压出现 0.7–1.2 m/s 的抖动，摩擦推动仍超过 1h（最深 7.5 mm），未采用；记录结果，ECO-1 验收场景待定 |
| 2026-10-27 | `8180ec3` | ECO-1 已完成：验收改为被推物体在无摩擦地面上平移（`PinnedBodyPushesAnotherWithoutPassingThrough`，最深节点 4.33 mm，前移 75.5 mm）；摩擦地面场景改为 `PinnedBodyPushesAnotherAlongAFloorWithFriction`（`[Ignore]`，BUG-8），并记录降低推动高度、加长被推物体后仍穿透的结果。删去 `PinnedBodyPushesAnotherAlongAFrictionlessFloor`（与新的验收测试相同） |
| 2026-10-27 | `3fd3216` | 补全变更记录中的提交号；PERF-4 改为部分完成（batchmode 已运行，Player 与 Quest 未测） |
| 2026-10-27 | `cf06c3a` | HAP-2 已完成：约束对运动学节点的力在求解中按（切片, 帧）累加，加上帧上节点自身的重量；`TryGetFrameWrench`、`TryGetGrabWrench`、`TryGetPinWrench`，接口草案改为已实现的签名（含 HAP-1 的 `TrySumColliderWrench`）。新增 EditMode 测试 2 个；基准测试 16 组 4 个阶段的 `positionsHash` 与修改前一致 |
| 2026-10-27 | `0defc6c` | HAP-3 已完成：新增 EditMode 测试 3 个（力平衡、球形压头与 Hertz 解比较、力逐位一致）；记录 lattice 刚度随 μΔt²/(ρh²) 降低、单精度坐标舍入对 mN 级力的影响；CFG-10 的依赖已完成。运行时代码未改动 |
| 2026-10-27 | `a9984ae` | 补充核对中发现但未列出的缺口：新增 BUG-9（刀刃前方的表面外棱不断开）、MED-17（器械接触的位置与细器械）；MED-10 补充只有一个抓取帧、帧只有平移；CFG-7 补充碰撞体查询上限 64 与物体间接触的常量；CFG-9 补充交互场景 S8–S11；ECO-8 补充快照不含碰撞体位姿与刚体状态；HAP-1 的验收改为与实现一致的力矩参考点 |
| 2026-10-27 | `28b6782` | HAP-4 已完成：每步为接触中的碰撞体发布局部线性力模型，K = −k N（接触法向张量乘沿运动方向的差分刚度）；节点 job 累加接触法向张量（`ApxWrench.ContactNormals`），`SoftBodySimulation` 增加 `SimulatedTime`、`ShapeMeanPosition`。新增 EditMode 测试 2 个；基准测试 16 组 4 个阶段的 `positionsHash` 与修改前一致 |
| 2026-10-27 | `0ba4a57` | 复核 HAP-4 与上一次 TODO 补充：HAP-4 的验收改为已采用的 K 的求法并写明测试条件，限制补上体素 5 mm 时的误差；BUG-8 修改前后基准测试 16 组 4 个阶段（rest、drop、cut、afterCut）的 `positionsHash` 与 `20c1e5e` 一致（此前只对比了 rest、afterCut） |
| 2026-10-27 | `d9fa515` | 文件行号改以 `0ba4a57` 为准：HAP-2、HAP-4 之后移动的 MED-4、SUT-1、CFG-7 中的行号；CFG-6 的已有派生量补充 `SimulatedTime` |
| 2026-10-27 | `0012562` | 补充核对中发现但未列出的缺口：新增 BUG-10（开放网格初始化时没有提示）、ECO-9（整体移动与 Transform 变化）、MED-18（切下组织块的管理）与第 6 节发布与工程（ENG-1 至 ENG-4：UPM 包、自动运行测试、基准测试基线、标准场景截图与里程碑验收）；CFG-7 补充钉住区域最多 31 个、超出部分没有提示；建议顺序改为第 7 节 |
| 2026-10-27 | `e5fb16e` | ENG-4 部分完成：按 `7389533` 在 batchmode 中重新截取 S1–S6、S8–S11，更新 `docs/images`、README 与 ROADMAP“当前结果”列；截图工具新增 S3b（拉开后侧三块）；M1、M2 的人工验收未完成 |
| 2026-10-27 | `92bf624` | ENG-5 已完成：交互工具模块（相机、可调朝向的手术刀、镊子、测量、调试显示、暂停与单步），两个演示场景改用；`ApxScalpel` 增加本刀路断棱数与上一次扫掠（只用于显示）；新增 EditMode 测试 6 个；截图工具新增 D1–D3 |
| 2026-10-27 | `fff7e00` | BUG-11 已完成：手术刀的指针平面在刀路内固定，刀以稳定绳（默认 8 mm）跟随指针；侧别断开的棱按两端到刀面的距离求切口点（`StrokeDistance`）。新增 EditMode 测试 2 个；更新有切口变化的 6 张截图。新增 BUG-12（刀路急转弯时断开刀未经过的棱，原因已确认）；ENG-5 的限制改为人工试用的结果 |
| 2026-10-27 | `b891564` | BUG-12 已完成：每一步另扫沿上一步刀路的尾部四边形，代替沿本步方向向后延长；节点按投影所在的最近扫掠三角形记侧别；刀刃前方的节点等刀经过时再补记侧别。新增 EditMode 测试 1 个，`WavyStrokePutsTheCutPointsOnTheBladePath` 增加断棱检查；基准测试 16 组块数不变，切割阶段哈希改变；更新有切口变化的 8 张截图。编辑器的 MCP 连接不可用，测试、基准测试与截图在工程副本上用 batchmode 运行 |
| 2026-10-28 | `1f9bcaa` | 基本形状上的柔体与交互作为第一个版本通过人工验收，记入 ROADMAP M1、M2，ENG-4 已完成；新增里程碑 M6（真实网格）与建议顺序第 8 项；更新本文件对照基准的说明，补全上一行提交号 |
| 2026-10-28 | `659be2a` | ENG-3 已完成：`Benchmarks/baseline.json` 入库（来自 `1f9bcaa`，16 组），基准测试结束时与基线比较 4 个阶段的哈希、块数、断棱数与最大的 4 块节点数，列出不一致的组；`-apxWriteBaseline`、`-apxBaseline`、`-apxFailOnBaselineDifference` 与菜单 Run Benchmarks and Write Baseline。基线与 BUG-12 修复后的 batchmode 结果 16 组逐项相同 |
| 2026-10-28 | `4bf44ba` | BUG-10 已完成：开放棱统计移到运行时（`MeshUtil.CountOpenEdges`，Burst），网格有边界棱时初始化与烘焙 blueprint 输出警告；新增 EditMode 测试 1 个；基准测试 16 组与基线一致。扫描版兔子 FBX 有 223 条边界棱 |
| 2026-10-28 | `3476b0f` | 真实网格摸底（DESIGN 6.2 节）：基准测试支持工程内的模型资产，报告记录表面更新的分阶段耗时（`SurfaceMesh.LastTimings`），刀面不再错过落地后倾倒的物体；用 3 个 FBX 测试模型运行，新增 BUG-13（不规则网格落地后轻节点抖动、不能休眠），更新 PERF-2、PERF-6、PERF-7 的现状 |
| 2026-10-28 | `3e03ef4` | PERF-10 已完成：网格有边界棱时格点内外按广义缠绕数判定（一阶近似的树），孔在 lattice 中封闭；闭合网格仍用射线奇偶；新增 EditMode 测试 1 个；基准测试 16 组与基线一致；扫描版兔子 48 的全切由 3 块（27981 / 3535 / 1）变为 2 块（16698 / 14133） |
| 2026-10-28 | `f3da8b2` | BUG-10 补充：开放棱统计改为焊接距离小于包围盒对角线 1e-6 倍的顶点（`MeshUtil.WeldTolerance`）。此前按位置精确焊接，椭球接缝（θ = 0 与 2π，相差约 1e-8 m）被判为开放，演示中的 Liver Lobe、Dropped Lobe 输出警告并改用缠绕数判定；`OpenMeshWarnsOnInitialize` 增加椭球为 0 条的检查 |
| 2026-10-28 | `2f1160b` | PERF-6 部分：表面网格的顶点分两个流，位置与法线每次更新一次写入（`SetVertexBufferData`），包围盒取 lattice 节点的包围盒，索引一次写入（`SetIndexBufferData`）；标准场景截图与修改前相比至多 122 个像素相差 1–3 个色阶 |
| 2026-10-28 | `2e140ca` | PERF-6 部分完成：触及切开体素的皮肤三角形在 Burst 中并行拆分（`SplitTrianglesJob`、`MergeSplitsJob`），去掉托管的拆分缓存；新增 EditMode 测试 `SplitJobMatchesTheManagedSplit`。同一时段前后对比：切割帧重建 p50 龙的 FBX 16 由 2231 ms 降至 85 ms、Fertility 16 由 1176 ms 降至 77 ms，每帧表面更新龙由 13–15 ms 降至 3.6–5.2 ms；节点位置哈希不变，标准场景截图不变 |
| 2026-10-28 | `39708d1` | BUG-13 原因已确认：同一块的折叠接触只按节点距离推开、不判断侧别，载荷下节点互相穿过后穿透被锁住、节点来回推挤；记录 C 形物体的复现与“按静止 SDF 缩小表面外节点的接触距离”的试验（抖动约减半，未消除，未采用）。运行时代码未改动 |
| 2026-10-28 | `8d525d8` | PERF-7 部分：射线奇偶判定按 (y, z) 行分桶、近表面距离按 z 层并行、表面细分移到 Burst，结果逐位不变；box 64 初始化 6.44 → 2.29 s，dragon 64 7.48 → 1.70 s。在独立工作树 `dev/m6-perf` 上修改，用工程副本以 batchmode 验证 |
| 2026-10-28 | `704a5a3` | PERF-6 第二部分：切面几何与可见副本移到 Burst，源顶点数据常驻，计时分出网格上传；box 64 切割帧重建 p50 63.9 → 22.6 ms，bunny 64 66.2 → 16.1 ms；高面数网格剩下网格上传与拆分 |
| 2026-10-28 | `9e5f31f` | PERF-2：分辨率 96、128 的基准结果（DESIGN 6.3 节）；新增 BUG-14（Fertility 128 全切不分离，原因未查明）。运行时代码未改动 |
| 2026-10-28 | `ac463a8` | 基准测试读取模型资产时包括根节点自身的旋转与缩放（单个网格的 FBX 把网格与轴向转换都放在根节点上）；更正 DESIGN 6.2、6.3 节：其中三个 FBX 模型的结果是按网格的原始坐标得到的 |
| 2026-10-28 | `7e38174` | 网格法线与三角形不符时（一致度低于 0.5）按三角形重新计算并警告：龙的 FBX 没有法线，Unity 导入时算出的全为 (0, 1, 0)，渲染为平涂；新增 EditMode 测试 1 个 |
| 2026-10-28 | `3b70eee` | 真实网格演示场景与标准场景 S12（`Assets/Samples/RealMeshDemo`）：三个 FBX 测试模型在桌面上，Fertility、龙全切后拉开，镊子提起兔子的耳朵；截图加入 `docs/images` 与 README，ROADMAP 加入 S12 |
| 2026-10-28 | `2d5e8f8` | BUG-13 部分：节点质量下限为体素质量的 1/16，只属于一个几乎为空的体素的角点不再在子步之间来回运动（bunny 48 关闭 `selfCollision` 落地 150 步：23 个节点 0.49 m/s → 0 个、0.002 m/s）。基准测试 16 组 4 个阶段的哈希全部改变（静止阶段也有约束修正的舍入），重写基线；全切块数 14 组不变，bunny 64 由 11 块变为 10 块且不再分成两大块（69541 / 3 / 2 / 1；落下 90 步后最大节点速度 3.3 m/s，刀经过时物体仍在大幅运动），dragon 48 由 4 块变为 8 块（两大块 7061 / 5620，另有不超过 57 个节点的碎块） |
| 2026-10-28 | `8ff475b` | BUG-13 部分完成：同一块的折叠接触改为按静止 SDF 判断侧别（`FoldCellsJob`、`BuildFoldHashJob`、`FindFoldCandidatesJob`、`FoldContactJob`，每个子步一次）；新增 EditMode 测试 `SaggingArmRestsOnTheBodyInsteadOfPassingThrough`（C 形物体）。C 形物体与 bunny 48 满足验收；基准测试 box 4 组、liver 16、dragon 16 的 4 个阶段哈希与上一提交相同，其余 10 组落下及以后的阶段改变（静止阶段相同），重写基线；全切块数与 BUG-13 之前的基线相比 bunny 64（11 → 6，不再分成两大块）与 dragon 64（2 → 4，另有两个碎块）改变，未满足验收中的这一项；更新 17 张标准场景截图（位置与形状略有变化，无新的缺陷） |
| 2026-10-28 | `2bcea79` | 合并 BUG-13（`f4f14e1`）后重新截取 S12，更新 `docs/images` 与 ROADMAP“当前结果”：静止后最大节点速度兔子 0.007 m/s、Fertility 与龙低于 0.001 m/s |
| 2026-10-28 | `64038e9` | 新增 BUG-15（第二刀经过已张开的切口时出现破面），部分完成：切面平面按所隔的两个副本分组拟合，同一皮肤块的顶点用相连的副本求值；削下碎块的问题待修复 |
| 2026-10-28 | `2098023` | BUG-15 已完成：后一刀不从已有切面上切下少于 16 个实节点的部分（`CutJob.KeepChips`）；新增 EditMode 测试 3 个（`RecutTests`）；单刀的基准测试 16 组与基线一致；更新有变化的 5 张截图。新增 BUG-16（第二刀擦过切口壁时留下细缝，原因未查明） |
| 2026-10-28 | `1959136` | BUG-16 部分完成：刀路结束时恢复贴着已有切面、未分离的断棱（`EndStroke`、`HealStrokeJob`）；`RecutTests` 新增 3 个用例，倾斜一例改为检查破洞；摆动、斜着切进壁面的缺口仍在。新增 BUG-17（切口贴着体素棱经过时切面上的三角形小孔，原因已确认）。单刀的基准测试 16 组与基线一致 |
| 2026-10-28 | `f5cc1d5` | BUG-17 已完成：同一块、在体素内只被落在切面内的断棱隔开的副本按一个区域生成切面（`SurfaceMesh.CutRegions`、`SeparatedNearby`）；新增 EditMode 测试 2 个（`SoftBodyTests`），`RebuildJobsMatchTheManagedRebuild` 的比较移到共用的辅助方法；演示场景单刀切面开口棱 24 → 0；基准测试 16 组与基线一致 |
| 2026-10-28 | — | 对照代码、文档与 Obi 7.2 全面核对：新增 ECO-10（事件）、ECO-11（空间查询）、ECO-12（渲染插值）、ECO-13（附着约束）、ENG-6（Unity 6）、ENG-7（求解后端接口）；ECO-9 增加局部参考空间求解与远离原点时的力误差；PERF-9 补充精度数据、候选方法与评估指标；PERF-5、PERF-8、MED-7、ECO-4、MED-13 补充依赖与顺序；建议顺序第 8 项收窄为分辨率不超过 64，新增第 9–11 项；ROADMAP 新增 M7 与“待决定”。运行时代码未改动 |
