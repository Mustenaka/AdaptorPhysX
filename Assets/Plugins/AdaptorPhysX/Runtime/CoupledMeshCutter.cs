using System;

namespace APEX.Native
{
    public sealed class CoupledMeshCutResult
    {
        internal CoupledMeshCutResult(
            RenderMeshCutResult render,
            ApxCutResult simulation,
            ApxCutDetails simulationDetails)
        {
            Render = render;
            Simulation = simulation;
            SimulationDetails = simulationDetails;
        }

        public RenderMeshCutResult Render { get; }

        public ApxCutResult Simulation { get; }

        public ApxCutDetails SimulationDetails { get; }
    }

    /// <summary>
    /// Commits one stable cut query to the coarse simulation mesh and the fine
    /// render mesh. The render candidate is calculated first but is only
    /// observable through the result after the native topology commit succeeds.
    /// </summary>
    public static class CoupledMeshCutter
    {
        public static CoupledMeshCutResult Cut(
            NativeWorld world,
            RenderMeshData renderMesh,
            ApxCutQuery query)
        {
            return Cut(world, renderMesh, query, query);
        }

        /// <summary>
        /// Uses two coordinate representations of the same geometric cut.
        /// The render query is normally target-local while the simulation
        /// query remains in world space.
        /// </summary>
        public static CoupledMeshCutResult Cut(
            NativeWorld world,
            RenderMeshData renderMesh,
            ApxCutQuery renderQuery,
            ApxCutQuery simulationQuery)
        {
            if (world == null)
            {
                throw new ArgumentNullException(nameof(world));
            }

            RenderMeshCutResult renderCandidate =
                RenderMeshCutter.Cut(renderMesh, renderQuery);
            ApxCutResult simulation = world.Cut(simulationQuery);
            ApxCutDetails details = world.GetLastCutDetails();
            return new CoupledMeshCutResult(renderCandidate, simulation, details);
        }
    }
}
