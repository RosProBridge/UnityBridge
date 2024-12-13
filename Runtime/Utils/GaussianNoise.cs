using System;

namespace Utils
{
    public static class GaussianNoise
    {
        /// <summary>
        /// Generates Gaussian noise using the Box-Muller transform.
        /// </summary>
        /// <param name="sigma">The standard deviation of the Gaussian distribution.</param>
        /// <returns>A double representing the Gaussian noise with the given standard deviation.</returns>
        public static double Generate(double sigma = 1.0d)
        {
            double u1 = UnityEngine.Random.value;
            double u2 = UnityEngine.Random.value;
            double randStdNormal = Math.Sqrt(-2.0d * Math.Log(u1)) * Math.Sin(2.0d * Math.PI * u2);
            return randStdNormal * sigma;
        }
    }
}