using System;

namespace APEX.Native
{
    /// <summary>
    /// Deterministically generated row-major cloth data shared by the Phase 1
    /// Unity demo and its gates. Construction validates every scalar before
    /// publishing any arrays.
    /// </summary>
    public sealed class PhaseOneClothWorkload
    {
        private PhaseOneClothWorkload(
            int columns,
            int rows,
            float spacing,
            ApxParticleDesc[] particles,
            ApxClothDistanceConstraintDesc[] clothConstraints,
            ApxRenderVertexBindingDesc[] renderBindings,
            uint[] triangleIndices)
        {
            Columns = columns;
            Rows = rows;
            Spacing = spacing;
            Particles = particles;
            ClothConstraints = clothConstraints;
            RenderBindings = renderBindings;
            TriangleIndices = triangleIndices;
        }

        public int Columns { get; }

        public int Rows { get; }

        public float Spacing { get; }

        public ApxParticleDesc[] Particles { get; }

        public ApxClothDistanceConstraintDesc[] ClothConstraints { get; }

        public ApxRenderVertexBindingDesc[] RenderBindings { get; }

        public uint[] TriangleIndices { get; }

        public static PhaseOneClothWorkload Create(
            int columns,
            int rows,
            float spacing,
            float warpCompliance,
            float weftCompliance,
            float breakThreshold)
        {
            if (columns < 3)
            {
                throw new ArgumentOutOfRangeException(nameof(columns));
            }
            if (rows < 3)
            {
                throw new ArgumentOutOfRangeException(nameof(rows));
            }
            ValidateFinitePositive(spacing, nameof(spacing));
            ValidateFiniteNonNegative(warpCompliance, nameof(warpCompliance));
            ValidateFiniteNonNegative(weftCompliance, nameof(weftCompliance));
            ValidateFinitePositive(breakThreshold, nameof(breakThreshold));

            int particleCount = checked(columns * rows);
            int warpCount = checked(rows * (columns - 1));
            int weftCount = checked((rows - 1) * columns);
            int clothCount = checked(warpCount + weftCount);
            int triangleIndexCount = checked((rows - 1) * (columns - 1) * 6);

            ApxParticleDesc[] particles = new ApxParticleDesc[particleCount];
            float halfWidth = (columns - 1) * spacing * 0.5F;
            float halfHeight = (rows - 1) * spacing * 0.5F;
            for (int row = 0; row < rows; ++row)
            {
                for (int column = 0; column < columns; ++column)
                {
                    int stableId = checked(row * columns + column);
                    particles[stableId] = new ApxParticleDesc(
                        new ApxVec3(
                            column * spacing - halfWidth,
                            halfHeight - row * spacing,
                            0.0F),
                        default,
                        row == 0 ? 0.0F : 1.0F);
                }
            }

            ApxClothDistanceConstraintDesc[] cloth =
                new ApxClothDistanceConstraintDesc[clothCount];
            int constraintIndex = 0;
            for (int row = 0; row < rows; ++row)
            {
                for (int column = 0; column + 1 < columns; ++column)
                {
                    uint left = checked((uint)(row * columns + column));
                    cloth[constraintIndex++] = new ApxClothDistanceConstraintDesc(
                        left,
                        left + 1U,
                        spacing,
                        warpCompliance,
                        breakThreshold,
                        ApxClothDirection.Warp);
                }
            }
            for (int row = 0; row + 1 < rows; ++row)
            {
                for (int column = 0; column < columns; ++column)
                {
                    uint top = checked((uint)(row * columns + column));
                    cloth[constraintIndex++] = new ApxClothDistanceConstraintDesc(
                        top,
                        checked(top + (uint)columns),
                        spacing,
                        weftCompliance,
                        breakThreshold,
                        ApxClothDirection.Weft);
                }
            }
            if (constraintIndex != cloth.Length)
            {
                throw new InvalidOperationException("Cloth workload count mismatch.");
            }

            ApxRenderVertexBindingDesc[] bindings =
                new ApxRenderVertexBindingDesc[particleCount];
            for (uint stableId = 0; stableId < (uint)particleCount; ++stableId)
            {
                uint neighborB = (stableId + 1U) % (uint)particleCount;
                uint neighborC = (stableId + 2U) % (uint)particleCount;
                bindings[stableId] = new ApxRenderVertexBindingDesc(
                    stableId,
                    neighborB,
                    neighborC,
                    1.0F,
                    0.0F,
                    0.0F);
            }

            uint[] indices = new uint[triangleIndexCount];
            int triangleIndex = 0;
            for (int row = 0; row + 1 < rows; ++row)
            {
                for (int column = 0; column + 1 < columns; ++column)
                {
                    uint topLeft = checked((uint)(row * columns + column));
                    uint topRight = topLeft + 1U;
                    uint bottomLeft = checked(topLeft + (uint)columns);
                    uint bottomRight = bottomLeft + 1U;
                    indices[triangleIndex++] = topLeft;
                    indices[triangleIndex++] = bottomLeft;
                    indices[triangleIndex++] = topRight;
                    indices[triangleIndex++] = topRight;
                    indices[triangleIndex++] = bottomLeft;
                    indices[triangleIndex++] = bottomRight;
                }
            }
            if (triangleIndex != indices.Length)
            {
                throw new InvalidOperationException("Render workload count mismatch.");
            }

            return new PhaseOneClothWorkload(
                columns,
                rows,
                spacing,
                particles,
                cloth,
                bindings,
                indices);
        }

        public ApxCutQuery CreateVerticalCut(int column, float radius)
        {
            if (column <= 0 || column + 1 >= Columns)
            {
                throw new ArgumentOutOfRangeException(nameof(column));
            }
            ValidateFiniteNonNegative(radius, nameof(radius));
            float x = Particles[column].Position.X;
            float top = Particles[0].Position.Y + Spacing;
            float bottom = Particles[(Rows - 1) * Columns].Position.Y - Spacing;
            return new ApxCutQuery(
                new ApxVec3(x, top, 0.0F),
                new ApxVec3(x, bottom, 0.0F),
                new ApxVec3(1.0F, 0.0F, 0.0F),
                radius);
        }

        private static void ValidateFinitePositive(float value, string name)
        {
            if (!IsFinite(value) || value <= 0.0F)
            {
                throw new ArgumentOutOfRangeException(name);
            }
        }

        private static void ValidateFiniteNonNegative(float value, string name)
        {
            if (!IsFinite(value) || value < 0.0F)
            {
                throw new ArgumentOutOfRangeException(name);
            }
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
