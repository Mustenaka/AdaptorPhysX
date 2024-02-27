using APEX.Common.Render;
using UnityEngine;

namespace APEX.Cloth
{
    public class ApexCloth : MonoBehaviour, IApexRender
    {
        public ApexRenderType GetRenderType()
        {
            return ApexRenderType.Cloth;
        }
    }
}