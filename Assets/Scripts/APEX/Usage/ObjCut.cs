using System.Collections.Generic;
using System;
using System.Linq;
using APEX.Native;
using UnityEngine;
using UnityEngine.Rendering;

public class ObjCut : MonoBehaviour
{
    #region 参数

    [Header("目标对象")]
    public GameObject target;
    public GameObject prefabPart;
    public Transform planeLocation;
    
    [Header("设置参数")]
    public float size = 5f;
    //刀片的厚度
    public float tickness = 0.005f;
    [Tooltip("使用 finite-segment 确定性精切；关闭时保留原有实体切割路径。")]
    public bool usePreciseRenderMeshCut = true;

    [Header("精切结果")]
    public uint lastCutTriangleCount;
    public uint lastCreatedSeamPairCount;
    
    [Header("填充")]
    public Material liverMat;
    
    [Header("其他 DEBUG")]
    //切割的面片
    public Vector3 planeNormal;
    public Vector3 planeNormal2;
    public Vector3 planePoint;
    public Vector3 planePoint2;

    //切断后上半部分的数据
    public List<Vector3> upVerts;
    public List<int> upTris;
    public List<Vector2> upUVs;
    public List<Vector3> upNormals;

    //切断后下半部分的数据
    public List<Vector3> downVerts;
    public List<int> downTris;
    public List<Vector2> downUVs;
    public List<Vector3> downNormals;

    //保存边缘切割后产生的多边形的数据,分为两块,一个上半部分,一个下半部分
    public List<Vector3> upcenterVerts;
    public List<Vector3> downcenterVerts;

    //切断后产生的2个部分
    GameObject topPart;
    GameObject bottomPart;

    //如果没有切断 ,生成一个新的实体模型
    GameObject allPart;
    GameObject midPart;
    
    public List<Vector3> midVerts;
    public List<int> midTris;
    public List<Vector2> midUVs;
    public List<Vector3> midNormals;

    /// <summary>
    /// 不与面片延长面相交的网格
    /// </summary>
    public List<Vector3> trisByNotcrossed;

    //网格的一些属性,可在判断时重复使用
    public Vector3[] triVerts;
    public Vector2[] triUvs;
    public Vector3[] triNormals;

    #endregion

    /// <summary>
    /// 辅助函数
    /// </summary>
    private void OnDrawGizmos()
    {
        var cacheColor = Gizmos.color;
        if (planeLocation == null) return;
        if (target == null) return;

        Mesh targetMesh = target.GetComponent<MeshFilter>().sharedMesh;
        Vector3[] verts = targetMesh.vertices;
        int[] tris = targetMesh.triangles;
        //TODO
        //面片旋转信息不回应
        for (int i = 0; i < tris.Length - 1; i += 3)
        {
            var a = GetIntersectPoint(planeLocation.forward, planeLocation.position, verts[tris[i]],
                verts[tris[i + 1]]);
            if (a != null)
            {
                var matrix = Matrix4x4.TRS(planeLocation.position,
                    Quaternion.FromToRotation(planeLocation.forward, Vector3.zero), planeLocation.localScale);
                var temp = matrix.inverse.MultiplyPoint(a.Value);
                var sizeHalf = size * 0.5f;

                if (temp.x <= sizeHalf
                    && temp.x >= -sizeHalf
                    && temp.y <= sizeHalf
                    && temp.y >= -sizeHalf)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(a.Value, 0.015f);
                    Gizmos.DrawLine(verts[tris[i]], verts[tris[i + 1]]);
                }
            }

            var b = GetIntersectPoint(planeLocation.forward, planeLocation.position, verts[tris[i + 1]],
                verts[tris[i + 2]]);
            if (b != null)
            {
                var matrix = Matrix4x4.TRS(planeLocation.position,
                    Quaternion.FromToRotation(planeLocation.forward, Vector3.zero), planeLocation.localScale);
                var temp = matrix.inverse.MultiplyPoint(b.Value);
                var sizeHalf = size * 0.5f;

                if (temp.x <= sizeHalf
                    && temp.x >= -sizeHalf
                    && temp.y <= sizeHalf
                    && temp.y >= -sizeHalf)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(b.Value, 0.015f);
                    Gizmos.DrawLine(verts[tris[i + 1]], verts[tris[i + 2]]);
                }
            }

            var c = GetIntersectPoint(planeLocation.forward, planeLocation.position, verts[tris[i + 2]],
                verts[tris[i]]);
            if (c != null)
            {
                var matrix = Matrix4x4.TRS(planeLocation.position,
                    Quaternion.FromToRotation(planeLocation.forward, Vector3.zero), planeLocation.localScale);
                var temp = matrix.inverse.MultiplyPoint(c.Value);
                var sizeHalf = size * 0.5f;

                if (temp.x <= sizeHalf
                    && temp.x >= -sizeHalf
                    && temp.y <= sizeHalf
                    && temp.y >= -sizeHalf)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(c.Value, 0.015f);
                    Gizmos.DrawLine(verts[tris[i + 2]], verts[tris[i]]);
                }
            }

            //刀口和三角面的交点
            Mesh mesh = new Mesh();
            //面需要的点
            List<Vector3> vertices = new List<Vector3>();
            //生成三边面时用到的vertices和index
            List<int> triangles = new List<int>();

            vertices.Add(verts[tris[i]]);
            vertices.Add(verts[tris[i + 1]]);
            vertices.Add(verts[tris[i + 2]]);
            triangles.Add(0);
            triangles.Add(1);
            triangles.Add(2);
            mesh.vertices = vertices.ToArray();
            mesh.triangles = triangles.ToArray();
            //重新计算顶点和法线
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
        }

        var cacheMatrix = Gizmos.matrix;
        Gizmos.matrix = Matrix4x4.TRS(planeLocation.position, planeLocation.rotation, Vector3.one);
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(size, size, 0f));
        Gizmos.matrix = cacheMatrix;
        Gizmos.color = cacheColor;
    }

    void Start()
    {
        triVerts = new Vector3[3];
        triUvs = new Vector2[3];
        triNormals = new Vector3[3];
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            Cut();
        }
    }

    public void Cut()
    {
        if (target == null)
            return;
        if (usePreciseRenderMeshCut)
        {
            CutPreciseRenderMesh();
            return;
        }
        CutMesh();
        GC.Collect();
    }

    // [Header("Extern")] 

    void CutPreciseRenderMesh()
    {
        MeshFilter meshFilter = target.GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null)
            return;

        Mesh source = meshFilter.sharedMesh;
        Vector3[] sourcePositions = source.vertices;
        Vector3[] sourceNormals = source.normals;
        Vector2[] sourceUvs = source.uv;
        int[] sourceIndices = source.triangles;
        RenderMeshVertex[] vertices = new RenderMeshVertex[sourcePositions.Length];
        for (int index = 0; index < sourcePositions.Length; ++index)
        {
            Vector3 normal = sourceNormals.Length == sourcePositions.Length
                ? sourceNormals[index]
                : Vector3.zero;
            Vector2 uv = sourceUvs.Length == sourcePositions.Length
                ? sourceUvs[index]
                : Vector2.zero;
            vertices[index] = new RenderMeshVertex(
                ToApx(sourcePositions[index]),
                ToApx(normal),
                uv.x,
                uv.y);
        }

        uint[] indices = new uint[sourceIndices.Length];
        for (int index = 0; index < sourceIndices.Length; ++index)
        {
            indices[index] = checked((uint)sourceIndices[index]);
        }

        Transform blade = planeLocation != null ? planeLocation : transform;
        float halfLength = Mathf.Max(0.0F, size) * 0.5F;
        Vector3 localStart = target.transform.InverseTransformPoint(
            blade.position - blade.right * halfLength);
        Vector3 localEnd = target.transform.InverseTransformPoint(
            blade.position + blade.right * halfLength);
        Vector3 localSideNormal =
            target.transform.InverseTransformDirection(blade.forward).normalized;
        Vector3 scale = target.transform.lossyScale;
        float maximumScale = Mathf.Max(
            Mathf.Abs(scale.x),
            Mathf.Max(Mathf.Abs(scale.y), Mathf.Abs(scale.z)));
        float localRadius = Mathf.Max(0.0F, tickness) /
            Mathf.Max(maximumScale, 1.0e-6F);

        RenderMeshCutResult result = RenderMeshCutter.Cut(
            new RenderMeshData(vertices, indices),
            new ApxCutQuery(
                ToApx(localStart),
                ToApx(localEnd),
                ToApx(localSideNormal),
                localRadius));

        RenderMeshVertex[] resultVertices = result.Mesh.Vertices;
        Vector3[] positions = new Vector3[resultVertices.Length];
        Vector3[] normals = new Vector3[resultVertices.Length];
        Vector2[] uvs = new Vector2[resultVertices.Length];
        for (int index = 0; index < resultVertices.Length; ++index)
        {
            positions[index] = ToUnity(resultVertices[index].Position);
            normals[index] = ToUnity(resultVertices[index].Normal);
            uvs[index] = new Vector2(resultVertices[index].U, resultVertices[index].V);
        }

        uint[] resultIndices = result.Mesh.Indices;
        int[] triangles = new int[resultIndices.Length];
        for (int index = 0; index < resultIndices.Length; ++index)
        {
            triangles[index] = checked((int)resultIndices[index]);
        }

        Mesh preciseMesh = new Mesh
        {
            name = source.name + "_PreciseCut",
            indexFormat = resultVertices.Length > ushort.MaxValue
                ? IndexFormat.UInt32
                : IndexFormat.UInt16,
            vertices = positions,
            normals = normals,
            uv = uvs,
            triangles = triangles,
        };
        preciseMesh.RecalculateBounds();
        meshFilter.sharedMesh = preciseMesh;
        MeshCollider meshCollider = target.GetComponent<MeshCollider>();
        if (meshCollider != null)
        {
            meshCollider.sharedMesh = preciseMesh;
        }

        lastCutTriangleCount = result.CutTriangleCount;
        lastCreatedSeamPairCount = result.CreatedSeamPairCount;
    }

    static ApxVec3 ToApx(Vector3 value)
    {
        return new ApxVec3(value.x, value.y, value.z);
    }

    static Vector3 ToUnity(ApxVec3 value)
    {
        return new Vector3(value.X, value.Y, value.Z);
    }
    
    void CutMesh()
    {
        //transform.gameObject.SetActive(false);
        planeNormal = (-transform.forward).normalized;
        planeNormal2 = (1 - tickness) * (-transform.forward).normalized;
        planePoint = (transform.position);
        planePoint2 = (transform.position -= transform.forward * tickness);
        Mesh targetMesh = target.GetComponent<MeshFilter>().sharedMesh;

        upcenterVerts = new List<Vector3>();
        downcenterVerts = new List<Vector3>();
        int[] tris = targetMesh.triangles;
        Vector2[] uvs = targetMesh.uv;
        Vector3[] verts = targetMesh.vertices;
        Vector3[] normals = targetMesh.normals;

        upVerts = new List<Vector3>();
        upTris = new List<int>();
        upUVs = new List<Vector2>();
        upNormals = new List<Vector3>();

        downVerts = new List<Vector3>();
        downTris = new List<int>();
        downUVs = new List<Vector2>();
        downNormals = new List<Vector3>();

        topPart = Instantiate(prefabPart);
        bottomPart = Instantiate(prefabPart);

        midVerts = new List<Vector3>();
        midTris = new List<int>();
        midUVs = new List<Vector2>();
        midNormals = new List<Vector3>();

        allPart = Instantiate(prefabPart);
        midPart = Instantiate(prefabPart);

        trisByNotcrossed = new List<Vector3>();
        //模型中遍历三角形 
        for (int i = 0; i < tris.Length; i += 3)
        {
            Vector3 worldp1 = target.transform.TransformPoint(verts[tris[i]]);
            Vector3 worldp2 = target.transform.TransformPoint(verts[tris[i + 1]]);
            Vector3 worldp3 = target.transform.TransformPoint(verts[tris[i + 2]]);

            Vector2 uv1 = uvs[tris[i]];
            Vector2 uv2 = uvs[tris[i + 1]];
            Vector2 uv3 = uvs[tris[i + 2]];

            Vector3 normal1 = target.transform.TransformVector(normals[tris[i]]);
            Vector3 normal2 = target.transform.TransformVector(normals[tris[i + 1]]);
            Vector3 normal3 = target.transform.TransformVector(normals[tris[i + 2]]);

            //3个点是否与面片的延长面相交
            bool[] intersected = DoesTriIntersectPlane(worldp1, worldp2, worldp3);
            //筛选出与面相交的一圈三角面
            if (intersected[0] || intersected[1] || intersected[2])
            {
                #region 每一个三角面的属性

                triVerts[0] = worldp1;
                triVerts[1] = worldp2;
                triVerts[2] = worldp3;

                triUvs[0] = uv1;
                triUvs[1] = uv2;
                triUvs[2] = uv3;

                triNormals[0] = normal1;
                triNormals[1] = normal2;
                triNormals[2] = normal3;

                #endregion

                //如果面未与面片相交,添加到网格中
                if (GetCrossTris(triVerts))
                {
                    midVerts.Add(allPart.transform.InverseTransformPoint(worldp1));
                    midVerts.Add(allPart.transform.InverseTransformPoint(worldp2));
                    midVerts.Add(allPart.transform.InverseTransformPoint(worldp3));
                    midTris.Add(midVerts.Count - 3);
                    midTris.Add(midVerts.Count - 2);
                    midTris.Add(midVerts.Count - 1);
                    midUVs.Add(uv1);
                    midUVs.Add(uv2);
                    midUVs.Add(uv3);
                    midNormals.Add(allPart.transform.InverseTransformVector(normal1));
                    midNormals.Add(allPart.transform.InverseTransformVector(normal2));
                    midNormals.Add(allPart.transform.InverseTransformVector(normal3));
                }
                else
                {
                    //处理相交部分的三角形
                    HandleIntersectionPoints(intersected, triVerts, triUvs, triNormals);
                }
            }
            else
            {
                if (Mathf.Sign(Vector3.Dot(planeNormal, (worldp1 - planePoint))) > 0)
                {
                    upVerts.Add(topPart.transform.InverseTransformPoint(worldp1));
                    upVerts.Add(topPart.transform.InverseTransformPoint(worldp2));
                    upVerts.Add(topPart.transform.InverseTransformPoint(worldp3));
                    upTris.Add(upVerts.Count - 3);
                    upTris.Add(upVerts.Count - 2);
                    upTris.Add(upVerts.Count - 1);
                    upUVs.Add(uv1);
                    upUVs.Add(uv2);
                    upUVs.Add(uv3);
                    upNormals.Add(topPart.transform.InverseTransformVector(normal1));
                    upNormals.Add(topPart.transform.InverseTransformVector(normal2));
                    upNormals.Add(topPart.transform.InverseTransformVector(normal3));
                }
                else
                {
                    downVerts.Add(bottomPart.transform.InverseTransformPoint(worldp1));
                    downVerts.Add(bottomPart.transform.InverseTransformPoint(worldp2));
                    downVerts.Add(bottomPart.transform.InverseTransformPoint(worldp3));
                    downTris.Add(downVerts.Count - 3);
                    downTris.Add(downVerts.Count - 2);
                    downTris.Add(downVerts.Count - 1);
                    downUVs.Add(uv1);
                    downUVs.Add(uv2);
                    downUVs.Add(uv3);
                    downNormals.Add(bottomPart.transform.InverseTransformVector(normal1));
                    downNormals.Add(bottomPart.transform.InverseTransformVector(normal2));
                    downNormals.Add(bottomPart.transform.InverseTransformVector(normal3));
                }
            }
        }

        //确定下部分的中心点
        Vector3 downcenter = Vector3.zero;
        for (int i = 0; i < downcenterVerts.Count; i++)
        {
            downcenter += downcenterVerts[i];
        }
        downcenter /= downcenterVerts.Count;
        
        //确定上部分中心点
        Vector3 upcenter = Vector3.zero;
        for (int i = 0; i < upcenterVerts.Count; i++)
        {
            upcenter += upcenterVerts[i];
        }
        upcenter /= upcenterVerts.Count;
        
        //如果是未切断模型,开始寻找最远处的2个点,并将这两个点与下部分的点合在一起
        if (trisByNotcrossed.Count > 0)
        {
            List<float> DisUp = new List<float>();
            List<float> DisUptmp = new List<float>();
            
            //寻找上部分的离中心点最长的2个点的下标
            for (int i = 0; i < upcenterVerts.Count; i++)
            {
                DisUptmp.Add(Vector3.Distance(upcenter, upcenterVerts[i]));
                DisUp.Add(Vector3.Distance(upcenter, upcenterVerts[i]));
            }

            DisUptmp.Sort();
            int maxIndex1 = DisUp.IndexOf(DisUptmp[DisUptmp.Count - 1]);
            int maxIndex2 = DisUp.IndexOf(DisUptmp[DisUptmp.Count - 2]);

            //-------------------------------------------------------------
            List<float> DisDown = new List<float>();
            List<float> DisDowntmp = new List<float>();
            for (int i = 0; i < downcenterVerts.Count; i++)
            {
                DisDowntmp.Add(Vector3.Distance(downcenter, downcenterVerts[i]));
                DisDown.Add(Vector3.Distance(downcenter, downcenterVerts[i]));
            }

            DisDowntmp.Sort();
            int maxIndex3 = DisDown.IndexOf(DisDowntmp[DisDowntmp.Count - 1]);
            int maxIndex4 = DisDown.IndexOf(DisDowntmp[DisDowntmp.Count - 2]);

            int maxIndex5 = upVerts.IndexOf(upcenterVerts[maxIndex1]);
            int maxIndex6 = upVerts.IndexOf(upcenterVerts[maxIndex2]);

            upcenterVerts[maxIndex1] = downcenterVerts[maxIndex3];
            upcenterVerts[maxIndex2] = downcenterVerts[maxIndex4];

            upVerts[maxIndex5] = downcenterVerts[maxIndex3];
            upVerts[maxIndex6] = downcenterVerts[maxIndex4];
        }

        IOrderedEnumerable<Vector3> DownorderedInnerVerts;
        if (planeNormal.y != 0)
        {
            float normalDir = Mathf.Sign(planeNormal.y);
            DownorderedInnerVerts =
                downcenterVerts.OrderBy(x => normalDir * Mathf.Atan2((x - downcenter).z, (x - downcenter).x));
        }
        else
        {
            float normalDir = Mathf.Sign(planeNormal.z);
            DownorderedInnerVerts =
                downcenterVerts.OrderBy(x => normalDir * Mathf.Atan2((x - downcenter).x, (x - downcenter).y));
        }

        IOrderedEnumerable<Vector3> UporderdInnerVerts;
        if (planeNormal.y != 0)
        {
            float normalDir = Mathf.Sign(planeNormal.y);
            UporderdInnerVerts =
                upcenterVerts.OrderBy(x => normalDir * Mathf.Atan2((x - upcenter).z, (x - upcenter).x));
        }
        else
        {
            float normalDir = Mathf.Sign(planeNormal.z);
            UporderdInnerVerts =
                upcenterVerts.OrderBy(x => normalDir * Mathf.Atan2((x - upcenter).x, (x - upcenter).y));
        }

        //最终生成模型
        //切断
        if (trisByNotcrossed.Count == 0)
        {
            //处理相交空间
            HandleIntersectedZone(upVerts, upTris, upUVs, upNormals, UporderdInnerVerts, upcenter, true);
            HandleIntersectedZone(downVerts, downTris, downUVs, downNormals, DownorderedInnerVerts, downcenter, false);
            //切开的模型创建新的网格数据
            CreateParts(topPart, upVerts, upTris, upUVs, upNormals);
            topPart.name = "top";
            CreateParts(bottomPart, downVerts, downTris, downUVs, downNormals);
            bottomPart.name = "bottom";
            Destroy(allPart);
            Destroy(target);
            Debug.Log("切断");
            
            topPart.AddComponent<MeshCollider>();
            bottomPart.AddComponent<MeshCollider>();
            //Destroy(topPart); 
            //Destroy(bottomPart);
            
            //Part1.GetComponent<MeshRenderer>().material = liverMat;
            //Part2.GetComponent<MeshRenderer>().material = liverMat;
        }
        else
        {
            // 销毁现有的 Mesh 数据
            if (allPart.GetComponent<MeshFilter>().mesh != null)
            {
                Destroy(allPart.GetComponent<MeshFilter>().mesh);
            }
            
            allPart.name = "all";
            HandleIntersectedZone(upVerts, upTris, upUVs, upNormals, UporderdInnerVerts, upcenter, true);
            HandleIntersectedZone(downVerts, downTris, downUVs, downNormals, DownorderedInnerVerts, downcenter, false);
            
            // 创建各个部分
            CreateParts(topPart, upVerts, upTris, upUVs, upNormals);
            topPart.name = "top";
            CreateParts(bottomPart, downVerts, downTris, downUVs, downNormals);
            bottomPart.name = "bottom";
            CreateParts(midPart, midVerts, midTris, midUVs, midNormals);
            midPart.name = "mid";
            
            // 将创建的部分设置为 allPart 的子对象
            topPart.transform.SetParent(allPart.transform);
            bottomPart.transform.SetParent(allPart.transform);
            midPart.transform.SetParent(allPart.transform);
            
            //合并网格
            MeshFilter[] meshFilters = allPart.GetComponentsInChildren<MeshFilter>();
            CombineInstance[] combine = new CombineInstance[meshFilters.Length];
            int i = 0;
            
            while (i < meshFilters.Length)
            {
                combine[i].mesh = meshFilters[i].sharedMesh;
                combine[i].transform = meshFilters[i].transform.localToWorldMatrix;
                meshFilters[i].gameObject.SetActive(false);
                i++;
            }

            // 创建新网格并进行合并
            Mesh combinedMesh = new Mesh();
            combinedMesh.CombineMeshes(combine);
            
            // 清理 CombineInstance 数据
            for (int j = 0; j < combine.Length; j++)
            {
                combine[j].mesh = null;  // 清空对原始网格的引用
            }
            
            // allPart.GetComponent<MeshFilter>().mesh = new Mesh();
            // allPart.GetComponent<MeshFilter>().mesh.CombineMeshes(combine);
            
            // 将合并后的网格应用到 allPart 的 MeshFilter
            allPart.GetComponent<MeshFilter>().mesh = combinedMesh;
            allPart.gameObject.SetActive(true);
            
            // 重新为 allPart 添加 MeshCollider
            if (allPart.GetComponent<MeshCollider>())
            {
                Destroy(allPart.GetComponent<MeshCollider>());
            }
            allPart.AddComponent<MeshCollider>();
            
            // 重置 allPart 位置，并设置 tag
            allPart.transform.position = new Vector3(0, 0, 0);
            allPart.transform.tag = "Cut";

            // Destroy(allPart.GetComponent<MeshFilter>().sharedMesh);
            Destroy(target);
            allPart.transform.gameObject.SetActive(true);
            target = allPart;
            Debug.Log("未切断");
        }
    }

    //补面
    void HandleIntersectedZone(List<Vector3> partVerts, List<int> partTris, List<Vector2> partUvs,
        List<Vector3> partNormals, IOrderedEnumerable<Vector3> orderedInnerVerts, Vector3 center, bool top)
    {
        List<int> centerTris = new List<int>();

        int sizeVertsBeforeCenter = partVerts.Count;
        Vector3 centerTmp = center;

        partVerts.AddRange(orderedInnerVerts);
        partVerts.Add(center);

        if (top)
        {
            for (int i = sizeVertsBeforeCenter; i < partVerts.Count - 1; i++)
            {
                centerTris.Add(i);
                centerTris.Add(i + 1);
                centerTris.Add(partVerts.Count - 1);
            }

            centerTris.Add(partVerts.Count - 1);
            centerTris.Add(partVerts.Count - 2);
            centerTris.Add(sizeVertsBeforeCenter);
        }
        else
        {
            for (int i = sizeVertsBeforeCenter; i < partVerts.Count - 1; i++)
            {
                centerTris.Add(i);
                centerTris.Add(partVerts.Count - 1);
                centerTris.Add(i + 1);
            }

            centerTris.Add(partVerts.Count - 2);
            centerTris.Add(partVerts.Count - 1);
            centerTris.Add(sizeVertsBeforeCenter);
        }

        partTris.AddRange(centerTris);

        Vector3 normal;
        if (top)
            normal = topPart.transform.InverseTransformVector(-planeNormal);
        else
            normal = bottomPart.transform.InverseTransformVector(planeNormal);
        for (int i = sizeVertsBeforeCenter; i < partVerts.Count; i++)
        {
            partUvs.Add(new Vector2(0, 0));
            partNormals.Add(normal.normalized * 3);
        }
    }

    //创建新的物体
    void CreateParts(GameObject part, List<Vector3> partVerts, List<int> partTris, List<Vector2> partUvs,
        List<Vector3> partNormals)
    {
        //Debug.Log("模型名称:" + part.name + "*模型顶点数:" + partVerts.Count + "+模型三角面数:" + partTris.Count + "*模型UV数:" + partUvs.Count + "*模型Normal数:" + partNormals.Count);
        //Debug.LogWarning("UV+Normal 应该相等");
        Mesh partMesh = part.GetComponent<MeshFilter>().mesh;
        partMesh.Clear();
        partMesh.vertices = partVerts.ToArray();
        partMesh.triangles = partTris.ToArray();
        partMesh.uv = partUvs.ToArray();
        partMesh.normals = partNormals.ToArray();
        partMesh.RecalculateBounds();
        //partMesh.RecalculateNormals();
        part.GetComponent<Renderer>().material = target.GetComponent<Renderer>().material;
    }

    //3个点构成的面是否与面板的延长线相交
    bool[] DoesTriIntersectPlane(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        float upOrDown = Mathf.Sign(Vector3.Dot(planeNormal, p1 - planePoint));
        float upOrDown2 = Mathf.Sign(Vector3.Dot(planeNormal, p2 - planePoint));
        float upOrDown3 = Mathf.Sign(Vector3.Dot(planeNormal, p3 - planePoint));

        bool intersect1 = upOrDown != upOrDown2;
        bool intersect2 = upOrDown2 != upOrDown3;
        bool intersect3 = upOrDown != upOrDown3;

        bool[] intersections = { intersect1, intersect2, intersect3 };

        return intersections;
    }

    //处理模型相交三角形的三个点
    void HandleIntersectionPoints(bool[] intersections, Vector3[] verts, Vector2[] uvs, Vector3[] normals)
    {
        //上下部分的网格
        List<Vector3> tmpUpVerts = new List<Vector3>();
        List<Vector3> tmpDownVerts = new List<Vector3>();
        //通过数字的正反判断在plane的哪一面
        float upOrDown = Mathf.Sign(Vector3.Dot(planeNormal, verts[0] - planePoint));
        float upOrDown2 = Mathf.Sign(Vector3.Dot(planeNormal, verts[1] - planePoint));
        float upOrDown3 = Mathf.Sign(Vector3.Dot(planeNormal, verts[2] - planePoint));

        if (intersections[0])
        {
            AddToCorrectSideList(upOrDown, 0, 1, verts, uvs, normals, tmpUpVerts, tmpDownVerts);
        }

        if (intersections[1])
        {
            AddToCorrectSideList(upOrDown2, 1, 2, verts, uvs, normals, tmpUpVerts, tmpDownVerts);
        }

        if (intersections[2])
        {
            AddToCorrectSideList(upOrDown3, 2, 0, verts, uvs, normals, tmpUpVerts, tmpDownVerts);
        }

        //处理相交部分三角形 
        HandleTriOrder(tmpUpVerts, tmpDownVerts);
    }

    //设置未与面片相交的点
    bool GetCrossTris(Vector3[] verts)
    {
        var a = GetIntersectPoint(planeLocation.forward, planeLocation.position, verts[0], verts[1]);
        if (a != null)
        {
            var matrix = Matrix4x4.TRS(planeLocation.position,
                Quaternion.FromToRotation(planeLocation.forward, Vector3.zero), planeLocation.localScale);
            var temp = matrix.inverse.MultiplyPoint(a.Value);
            var sizeHalf = size * 0.5f;

            if (temp.x <= sizeHalf
                && temp.x >= -sizeHalf
                && temp.y <= sizeHalf
                && temp.y >= -sizeHalf)
            {
                //Debug.Log("a相交的点"+a .Value );
                return false;
            }
        }

        var b = GetIntersectPoint(planeLocation.forward, planeLocation.position, verts[1], verts[2]);
        if (b != null)
        {
            var matrix = Matrix4x4.TRS(planeLocation.position,
                Quaternion.FromToRotation(planeLocation.forward, Vector3.zero), planeLocation.localScale);
            var temp = matrix.inverse.MultiplyPoint(b.Value);
            var sizeHalf = size * 0.5f;

            if (temp.x <= sizeHalf
                && temp.x >= -sizeHalf
                && temp.y <= sizeHalf
                && temp.y >= -sizeHalf)
            {
                //Debug.Log("b相交的点" + b.Value);
                return false;
            }
        }

        var c = GetIntersectPoint(planeLocation.forward, planeLocation.position, verts[2], verts[0]);
        if (c != null)
        {
            var matrix = Matrix4x4.TRS(planeLocation.position,
                Quaternion.FromToRotation(planeLocation.forward, Vector3.zero), planeLocation.localScale);
            var temp = matrix.inverse.MultiplyPoint(c.Value);
            var sizeHalf = size * 0.5f;

            if (temp.x <= sizeHalf
                && temp.x >= -sizeHalf
                && temp.y <= sizeHalf
                && temp.y >= -sizeHalf)
            {
                //Debug.Log("c相交的点" + c.Value);
                return false;
            }
        }

        //所有切割延长面上没有被切割到的三角形list ,
        trisByNotcrossed.Add(verts[0]);
        trisByNotcrossed.Add(verts[1]);
        trisByNotcrossed.Add(verts[2]);
        return true;
    }

    //重心
    void HandleBaryCentric(Vector3 newPoint, ref Vector2 newUV, ref Vector3 newNormal, Vector3[] points, Vector2[] uvs,
        Vector3[] normals)
    {
        Vector3 f1 = points[0] - newPoint;
        Vector3 f2 = points[1] - newPoint;
        Vector3 f3 = points[2] - newPoint;
        // calculate the areas and factors (order of parameters doesn't matter):
        // 计算区域和因素
        float areaMainTri =
            Vector3.Cross(points[0] - points[1], points[0] - points[2]).magnitude; // main triangle area a
        float a1 = Vector3.Cross(f2, f3).magnitude / areaMainTri; // p1's triangle area / a
        float a2 = Vector3.Cross(f3, f1).magnitude / areaMainTri; // p2's triangle area / a 
        float a3 = Vector3.Cross(f1, f2).magnitude / areaMainTri; // p3's triangle area / a
        // find the uv corresponding to point f (uv1/uv2/uv3 are associated to p1/p2/p3):
        //找到f点对应的uv (uv1/uv2/uv3与p1/p2/p3相关联):  
        newNormal = normals[0] * a1 + normals[1] * a2 + normals[2] * a3;
        newUV = uvs[0] * a1 + uvs[1] * a2 + uvs[2] * a3;
    }

    /// <summary>
    /// 处理切开模型的三角形列表
    /// </summary>
    /// <param name="tmpUpVerts">上半部分的补面</param>
    /// <param name="tmpDownVerts">下半部分的补面</param>
    void HandleTriOrder(List<Vector3> tmpUpVerts, List<Vector3> tmpDownVerts)
    {
        int upLastInsert = upVerts.Count;
        int downLastInsert = downVerts.Count;

        //添加集合到指定集合的末尾
        downVerts.AddRange(tmpDownVerts);
        upVerts.AddRange(tmpUpVerts);
        upTris.Add(upLastInsert);
        upTris.Add(upLastInsert + 1);
        upTris.Add(upLastInsert + 2);

        if (tmpUpVerts.Count > 3)
        {
            upTris.Add(upLastInsert);
            upTris.Add(upLastInsert + 2);
            upTris.Add(upLastInsert + 3);
        }

        downTris.Add(downLastInsert);
        downTris.Add(downLastInsert + 1);
        downTris.Add(downLastInsert + 2);

        if (tmpDownVerts.Count > 3)
        {
            downTris.Add(downLastInsert);
            downTris.Add(downLastInsert + 2);
            downTris.Add(downLastInsert + 3);
        }
    }

    /// <summary>
    /// 添加到正确的列表中
    /// </summary>
    /// <param name="upOrDown">三角形的点在面片的上方还是下方 </param>
    /// <param name="pointIndex1">第一个点</param>
    /// <param name="pointIndex2">第二个点</param>
    /// <param name="verts">三角形的三个顶点数组</param>
    /// <param name="uvs">三角形的UV</param>
    /// <param name="normals">三角形的法线</param>
    /// <param name="top"></param>
    /// <param name="bottom"></param>
    void AddToCorrectSideList(float upOrDown, int pointIndex1, int pointIndex2, Vector3[] verts,
        Vector2[] uvs, Vector3[] normals, List<Vector3> top, List<Vector3> bottom)
    {
        Vector3 point1 = verts[pointIndex1];
        Vector3 point2 = verts[pointIndex2];
        Vector2 uv1 = uvs[pointIndex1];
        Vector2 uv2 = uvs[pointIndex2];
        Vector3 n1 = normals[pointIndex1];
        Vector3 n2 = normals[pointIndex2];

        Vector3 rayDir = (point2 - point1).normalized;
        //上下部分的比例
        float t = Vector3.Dot(planePoint - point1, planeNormal) / Vector3.Dot(rayDir, planeNormal);
        float t2 = Vector3.Dot(planePoint2 - point1, planeNormal2) / Vector3.Dot(rayDir, planeNormal2);
        Vector3 newVert = point1 + rayDir * t;
        Vector3 newVert2 = point1 + rayDir * t2;
        Vector2 newUv = new Vector2(0, 0);
        Vector3 newNormal = new Vector3(0, 0, 0);
        HandleBaryCentric(newVert, ref newUv, ref newNormal, verts, uvs, normals);

        Vector3 topNewVert = topPart.transform.InverseTransformPoint(newVert2);
        Vector3 botNewVert = bottomPart.transform.InverseTransformPoint(newVert);
        Vector3 topNewNormal = topPart.transform.InverseTransformVector(newNormal).normalized;
        Vector3 botNewNormal = bottomPart.transform.InverseTransformVector(newNormal).normalized;
        //第一个点在平面的上方
        if (upOrDown > 0)
        {
            point1 = topPart.transform.InverseTransformPoint(point1);
            point2 = bottomPart.transform.InverseTransformPoint(point2);
            n1 = topPart.transform.InverseTransformVector(n1).normalized;
            n2 = bottomPart.transform.InverseTransformVector(n2).normalized;

            if (!top.Contains(point1))
            {
                top.Add(point1);
                upUVs.Add(uv1);
                upNormals.Add(n1);
            }

            top.Add(topNewVert);
            upUVs.Add(newUv);
            upNormals.Add(topNewNormal);

            bottom.Add(botNewVert);
            downUVs.Add(newUv);
            downNormals.Add(botNewNormal);

            if (!bottom.Contains(point2))
            {
                bottom.Add(point2);
                downUVs.Add(uv2);
                downNormals.Add(n2);
            }

            upcenterVerts.Add(topNewVert);
            downcenterVerts.Add(botNewVert);
        }
        //第一个点在平面的下方
        else
        {
            point2 = topPart.transform.InverseTransformPoint(point2);
            point1 = bottomPart.transform.InverseTransformPoint(point1);
            n2 = topPart.transform.InverseTransformVector(n2).normalized;
            n1 = bottomPart.transform.InverseTransformVector(n1).normalized;

            top.Add(topNewVert);
            upUVs.Add(newUv);
            upNormals.Add(topNewNormal);

            if (!top.Contains(point2))
            {
                top.Add(point2);
                upUVs.Add(uv2);
                upNormals.Add(n2);
            }

            if (!bottom.Contains(point1))
            {
                bottom.Add(point1);
                downUVs.Add(uv1);
                downNormals.Add(n1);
            }

            bottom.Add(botNewVert);
            downUVs.Add(newUv);
            downNormals.Add(botNewNormal);
            upcenterVerts.Add(topNewVert);
            downcenterVerts.Add(botNewVert);
        }
    }

    //2点之间与面片相交的点,判断是否切断整个模型 受物体缩放影响
    Vector3? GetIntersectPoint(Vector3 planeNormal, Vector3 planePosition, Vector3 p0, Vector3 p1)
    {
        //
        var sign1 = Mathf.Sign(Vector3.Dot(planeNormal, planePosition - p0));
        var sign2 = Mathf.Sign(Vector3.Dot(planeNormal, planePosition - p1));
        if (Mathf.Approximately(sign1, sign2)) return null; //同侧异侧.

        var a = planeNormal.x;
        var b = planeNormal.y;
        var c = planeNormal.z;
        var d = -a * planePosition.x - b * planePosition.y - c * planePosition.z;

        var i0 = a * p0.x + b * p0.y + c * p0.z;
        var i1 = a * p1.x + b * p1.y + c * p1.z;
        var final_t = -(i1 + d) / (i0 - i1);

        var finalPoint = new Vector3()
        {
            x = p0.x * final_t + p1.x * (1 - final_t),
            y = p0.y * final_t + p1.y * (1 - final_t),
            z = p0.z * final_t + p1.z * (1 - final_t),
        };

        return finalPoint;
    }

    //计算直线与平面的交点
    private Vector3 GetIntersectWithLineAndPlane(Vector3 point, Vector3 direct, Vector3 planeNormal, Vector3 planePoint)
    {
        float d = Vector3.Dot(planePoint - point, planeNormal) / Vector3.Dot(direct.normalized, planeNormal);
        return d * direct.normalized + point;
    }

    //确定坐标是否在平面内
    private bool IsVecPosPlane(Vector3[] vecs, Vector3 pos)
    {
        float RadianValue = 0;
        Vector3 vecOld = Vector3.zero;
        Vector3 vecNew = Vector3.zero;
        for (int i = 0; i < vecs.Length; i++)
        {
            if (i == 0)
            {
                vecOld = vecs[i] - pos;
            }

            if (i == vecs.Length - 1)
            {
                vecNew = vecs[0] - pos;
            }
            else
            {
                vecNew = vecs[i + 1] - pos;
            }

            RadianValue += Mathf.Acos(Vector3.Dot(vecOld.normalized, vecNew.normalized)) * Mathf.Rad2Deg;
            vecOld = vecNew;
        }

        if (Mathf.Abs(RadianValue - 360) < 0.1f)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //如果刀口碰到了需要切割的物体
    private void OnTriggerEnter(Collider other)
    {
        if (other.transform.tag.Equals("Cut"))
        {
            target = other.transform.gameObject;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.transform.tag.Equals("Cut"))
        {
            allPart.transform.gameObject.SetActive(true);
            Destroy(target);
        }
    }
}
