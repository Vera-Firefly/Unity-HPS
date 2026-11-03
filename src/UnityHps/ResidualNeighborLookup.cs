using System.Runtime.CompilerServices;

namespace UnityHps;

// Calibration data is immutable and shared. Keep the legacy row/heap order,
// while scanning only its five search coordinates in contiguous storage.
internal static class ResidualNeighborLookup
{
    static readonly ConditionalWeakTable<double[][], Lazy<double[]>> Coordinates = new();
    static double[] Pack(double[][] points)
    {
        var result = new double[checked(points.Length * 5)];
        for (int i = 0; i < points.Length; i++)
            Array.Copy(points[i], 0, result, i * 5, 5);
        return result;
    }
    internal static PriorityQueue<(int Index, double Distance), double> Find(
        double[][] points, double[] query, int count)
    {
        if (count < 1 || count > points.Length) throw new ArgumentOutOfRangeException(nameof(count));
        var coordinates = Coordinates.GetValue(points, p => new Lazy<double[]>(() => Pack(p))).Value;
        var nearest = new PriorityQueue<(int Index, double Distance), double>();
        double q0 = query[0], q1 = query[1], q2 = query[2], q3 = query[3], q4 = query[4];
        for (int i = 0, offset = 0; i < points.Length; i++)
        {
            double distance = 0, value = coordinates[offset++] - q0; distance += value * value;
            value = coordinates[offset++] - q1; distance += value * value;
            value = coordinates[offset++] - q2; distance += value * value;
            value = coordinates[offset++] - q3; distance += value * value;
            value = coordinates[offset++] - q4; distance += value * value;
            if (nearest.Count < count) nearest.Enqueue((i, distance), -distance);
            else if (distance < nearest.Peek().Distance)
            { nearest.Dequeue(); nearest.Enqueue((i, distance), -distance); }
        }
        return nearest;
    }
}
