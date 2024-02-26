using APEX.Common.RenderType;
using UnityEngine;

namespace APEX.Softbody
{
    public class ApexSoftbody : MonoBehaviour, IApexRender
    {
        public ApexRenderType GetRenderType()
        {
            return ApexRenderType.Softbody;
        }
    }
}