namespace g3
{
    public class Constants
    {
        /// <summary>
        /// DEFAULT_LINE_IDENTITY_TOLERANCE is coupled to default clipper resolution
        /// this is to ensure that "on line" vertices as output by clipper are treated as "on line" by other algorithms like convex hulls as well
        /// </summary>
        public const double DEFAULT_LINE_IDENTITY_TOLERANCE = 20d / Clipper2Wrapper.DefaultScale;
    }
}
