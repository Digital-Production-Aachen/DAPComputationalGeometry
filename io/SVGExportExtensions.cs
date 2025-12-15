using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;

namespace g3
{
    public static class SVGExportExtensions
    {
        public static string ExportAsSVG(this g3.Polygon2d polygon2D, string targetFile = null, SVGWriter.Style style = null)
        {
            var sVGWriter = StartExportAsSVG(polygon2D, style);
            return sVGWriter.WriteDebug(targetFile);
        }

        public static string ExportAsSVG(this IEnumerable<g3.Polygon2d> polygon2Ds, string targetFile = null, SVGWriter.Style style = null)
        {
            var sVGWriter = StartExportAsSVG(polygon2Ds, style);
            return sVGWriter.WriteDebug(targetFile);
        }

        public static string ExportWithAnimation(this g3.Polygon2d polygon2D, string targetFile = null, SVGWriter.Style style = null, double animationDurationInS = 10)
        {
            var polyStyle = style ?? SVGWriter.Style.Outline("black", 0.1f);
            var animationStyle = polyStyle.WithPathAnimation(polygon2D.ArcLength, animationDurationInS);
            var sVGWriter = StartExportAsSVG(polygon2D, polyStyle.WithStrokeOpacity(0.5));
            sVGWriter.AddPolygon(polygon2D.Duplicate(), animationStyle);
            return sVGWriter.WriteDebug(targetFile);
        }

        public static string ExportObjectsAsSVG(IEnumerable<object> objectsToExport, string targetFile = null, SVGWriter.Style style = null, double animationDurationInS = 10)
        {
            var writer = DefaultSVGWriter();
            var polyStyle = style ?? SVGWriter.Style.Outline("black", 0.1f);
            foreach (var obj in objectsToExport)
            {
                if (obj is null) continue;
                switch (obj)
                {
                    case IEnumerable<g3.Polygon2d>:
                        foreach(var poly in obj as IEnumerable<g3.Polygon2d>) 
                        {
                            writer.AddPolygonWithAnimation(poly, polyStyle, animationDurationInS);
                        }
                        break;
                    case g3.Polygon2d:
                        writer.AddPolygonWithAnimation(obj as g3.Polygon2d, polyStyle, animationDurationInS);
                        break;
                    case g3.Vector2d:
                        writer.AddPoint((obj as g3.Vector2d?).Value, polyStyle);
                        break;
                    case g3.PolyLine2d:
                        writer.AddPolyline(obj as g3.PolyLine2d, polyStyle);
                        break;
                    default:
                        break;
                }
            }
            return writer.WriteDebug(targetFile);

            
        }

        public static void AddPolygonWithAnimation(this SVGWriter writer, IEnumerable<g3.Polygon2d> polygons, SVGWriter.Style polyStyle, double animationDurationInS = 10)
        {
            if (polygons is null) return;
            foreach (var poly in polygons) writer.AddPolygonWithAnimation(poly, polyStyle, animationDurationInS);
        }

        public static void AddPolygonWithAnimation(this SVGWriter writer, g3.Polygon2d polygon, SVGWriter.Style polyStyle, double animationDurationInS = 10)
        {
            if (polygon is null) return;
            writer.AddPolygon(polygon, polyStyle.WithStrokeOpacity(0.5));
            var animationStyle = polyStyle.WithPathAnimation(polygon.ArcLength, animationDurationInS);
            writer.AddPolygon(polygon.Duplicate(), animationStyle);
        }

        public static SVGWriter StartExportAsSVG(this IEnumerable<g3.Polygon2d> polygon2Ds, SVGWriter.Style style = null)
        {
            if (polygon2Ds == null) return DefaultSVGWriter();
            var sVGWriter = StartExportAsSVG(polygon2Ds?.FirstOrDefault(), style);
            foreach (var poly in polygon2Ds.Skip(1))
            {
                if (style != null)
                    sVGWriter.AddPolygon(poly, style);
                else
                    sVGWriter.AddPolygon(poly);
            }
            return sVGWriter;
        }

        public static string ExportAsSVG(this GeneralPolygon2d generalPolygon, string targetFile = null, SVGWriter.Style style = null, SVGWriter.Style holeStyle = null)
        {
            if (generalPolygon == null) return "";
            var sVGWriter = StartExportAsSVG(generalPolygon, style, holeStyle);
            return sVGWriter.WriteDebug(targetFile);
        }

        public static string ExportAsSVG(this IEnumerable<GeneralPolygon2d> generalPolygons, string targetFile = null, SVGWriter.Style style = null, SVGWriter.Style holeStyle = null)
        {
            if (generalPolygons == null) return "";
            var sVGWriter = StartExportAsSVG(generalPolygons.First(), style, holeStyle);
            sVGWriter.AddGeneralPolygons(generalPolygons.Skip(1), style, holeStyle);
            return sVGWriter.WriteDebug(targetFile);
        }

        public static SVGWriter StartExportAsSVG(this IEnumerable<GeneralPolygon2d> generalPolygons, SVGWriter.Style style = null, SVGWriter.Style holeStyle = null)
        {
            if (generalPolygons == null) return DefaultSVGWriter();
            var sVGWriter = StartExportAsSVG(generalPolygons.FirstOrDefault(), style, holeStyle);
            sVGWriter.AddGeneralPolygons(generalPolygons.Skip(1), style, holeStyle);
            return sVGWriter;
        }

        public static SVGWriter StartExportAsSVG(this g3.Polygon2d polygon2D, SVGWriter.Style style = null)
        {
            SVGWriter sVGWriter = DefaultSVGWriter();
            if (polygon2D?.VertexCount > 0)
            {
                if (style != null)
                    sVGWriter.AddPolygon(polygon2D, style);
                else
                    sVGWriter.AddPolygon(polygon2D);
            }
            return sVGWriter;
        }

        public static SVGWriter DefaultSVGWriter()
        {
            SVGWriter sVGWriter = new SVGWriter();
            sVGWriter.SetDefaultLineWidth(0.1f);
            sVGWriter.Precision = 10;
            return sVGWriter;
        }

        public static SVGWriter StartExportAsSVG(this GeneralPolygon2d generalPolygon, SVGWriter.Style style = null, SVGWriter.Style holeStyle = null)
        {
            var sVGWriter = StartExportAsSVG(generalPolygon?.Outer, style);
            if(generalPolygon == null) return sVGWriter;
            foreach (var hole in generalPolygon.Holes)
            {
                if (holeStyle != null)
                    sVGWriter.AddPolygon(hole, holeStyle);
                else
                    sVGWriter.AddPolygon(hole, SVGWriter.Style.Outline("grey", 0.1f));
            }
            return sVGWriter;
        }

        public static void AddGeneralPolygon(this SVGWriter writer, GeneralPolygon2d generalPolygon, SVGWriter.Style style = null, SVGWriter.Style holeStyle = null)
        {
            if (generalPolygon == null) return;
            if (style != null)
                writer.AddPolygon(generalPolygon.Outer, style);
            else
                writer.AddPolygon(generalPolygon.Outer);

            foreach (var hole in generalPolygon.Holes)
            {
                if (holeStyle != null)
                    writer.AddPolygon(hole, holeStyle);
                else
                    writer.AddPolygon(hole, SVGWriter.Style.Outline("grey", 0.1f));
            }
        }

        public static void AddPolygons(this SVGWriter writer, IEnumerable<g3.Polygon2d> polygons, SVGWriter.Style style = null)
        {
            if (polygons == null) return;
            foreach (var poly in polygons)
            {
                if (style != null)
                    writer.AddPolygon(poly, style);
                else
                    writer.AddPolygon(poly);
            }
        }

        public static void AddBox(this SVGWriter writer, g3.Box2d box, SVGWriter.Style style = null)
        {
            if (style != null)
                writer.AddPolygon(box.ToPolygon2d(), style);
            else
                writer.AddPolygon(box.ToPolygon2d());
        }

        public static void AddVector(this SVGWriter writer, g3.Vector2d vector, SVGWriter.Style style = null) => AddVectors(writer, [vector], style);

        public static void AddVectors(this SVGWriter writer, IEnumerable<g3.Vector2d> vectors, SVGWriter.Style style = null)
        {
            foreach (var vector in vectors)
            {
                var line = new Segment2d(Vector2d.Zero, vector);
                if (style != null)
                    writer.AddLine(line, style);
                else
                    writer.AddLine(line);
            }
        }

        public static void AddGeneralPolygons(this SVGWriter writer, IEnumerable<GeneralPolygon2d> generalPolygons, SVGWriter.Style style = null, SVGWriter.Style holeStyle = null)
        {
            foreach (var genPoly in generalPolygons) writer.AddGeneralPolygon(genPoly, style, holeStyle);
        }

        public static string WriteDebug(this SVGWriter writer, string targetFile = null)
        {
            targetFile = ValidateFileName(targetFile, ".svg");
            writer.Write(targetFile);
            return targetFile;
        }

        private static string ValidateFileName(string targetFile, string extension)
        {
            if (targetFile == null)
            {
                targetFile = Path.Combine(Path.GetTempPath(), Guid.NewGuid().ToString());
            }
            var path = new FileInfo(targetFile);
            if (!Directory.Exists(Path.GetDirectoryName(targetFile))) path = new FileInfo(Path.Combine(Path.GetTempPath(), path.Name));
            if (path.Extension != extension)
            {
                targetFile = Path.Combine(path.DirectoryName, Path.GetFileNameWithoutExtension(path.Name) + extension);
            }
            int i = 0;
            while(new FileInfo(targetFile).Exists)
            {
                targetFile = Path.Combine(path.DirectoryName, Path.GetFileName(path.Name) + i++.ToString() + extension);
            }
            return targetFile;
        }

        public static void AddPoint(this SVGWriter sVGWriter, Vector2d point, SVGWriter.Style style, float size = 1f)
            => AddPoint(sVGWriter, point.x, point.y, style, size);        

        public static void AddPoint(this SVGWriter sVGWriter, double x, double y, SVGWriter.Style style, float size = 1f)
        {
            sVGWriter.AddLine(new Segment2d(new Vector2d(x - size, y + size), new Vector2d(x + size, y - size)), style);
            sVGWriter.AddLine(new Segment2d(new Vector2d(x - size, y - size), new Vector2d(x + size, y + size)), style);
        }

        /// <summary>
        /// starts exporting the given polygons as "explosion", meaning translated away from the coordinate center
        /// to create gaps between neighboring polygons. Optionally hulls for the polygons can be passed in.
        /// </summary>
        /// <param name="polygons"></param>
        /// <returns></returns>
        public static SVGWriter StartWriteAsExplosion(IList<Polygon2d> polygons, IList<Polygon2d> hulls = null)
        {
            if (hulls != null) Debug.Assert(polygons.Count == hulls.Count);
            var writer = new SVGWriter();
            for (int i = 0; i < polygons.Count(); i++)
            {
                var fragment = polygons[i].Duplicate();
                var centroid = fragment.Centroid();
                var translation = centroid / 10;
                fragment.Translate(translation);
                writer.AddPolygon(fragment, SVGWriter.Style.Filled("blue"));
                if (hulls is null) continue;
                var hull = hulls[i].Duplicate();
                hull.Translate(translation);
                writer.AddPolygon(hull, SVGWriter.Style.Outline("cyan", 0.2f));
            }
            return writer;
        }

        public static SVGWriter.Style AsDashed(this SVGWriter.Style style, double dashSize, double dashGapSize)
        {
            var dashed = new SVGWriter.Style();
            dashed.Fill = style.Fill;
            dashed.Stroke = $"{style.Stroke};stroke-dasharray:{dashSize.ToString(CultureInfo.InvariantCulture)},{dashGapSize.ToString(CultureInfo.InvariantCulture)}";
            dashed.Stroke_width = style.Stroke_width;
            return dashed;
        }

        public static SVGWriter.Style WithFillOpacity(this SVGWriter.Style style, double opacity)
        {
            var opaque = new SVGWriter.Style();
            opaque.Fill = $"{style.Fill};fill-opacity:{opacity.ToString(CultureInfo.InvariantCulture)}";
            opaque.Stroke = style.Stroke;
            opaque.Stroke_width = style.Stroke_width;
            return opaque;
        }

        public static SVGWriter.Style WithStrokeOpacity(this SVGWriter.Style style, double opacity)
        {
            var opaque = new SVGWriter.Style();
            opaque.Fill = style.Fill;
            opaque.Stroke = $"{style.Stroke};stroke-opacity:{opacity.ToString(CultureInfo.InvariantCulture)}";
            opaque.Stroke_width = style.Stroke_width;
            return opaque;
        }

        public static SVGWriter.Style WithPathAnimation(this SVGWriter.Style style, double animationPathLength, double animationDurationInS)
        {
            var animated = new SVGWriter.Style();
            var lengthStr = animationPathLength.ToString(CultureInfo.InvariantCulture);
            //SVG injection attack :-)
            animated.Fill = $"{style.Fill};stroke:{style.Stroke};stroke-width:{style.Stroke_width.ToString(CultureInfo.InvariantCulture)};stroke-dasharray:{lengthStr};stroke-dashoffset:{lengthStr};" +
                $"\"><animate attributeName=\"stroke-dashoffset\" from=\"{lengthStr}\" to=\"0\" dur=\"{animationDurationInS.ToString(CultureInfo.InvariantCulture)}s\" repeatCount=\"indefinite\"/></polygon>\r\n<polygon points=\"\" style=\"fill:none";
            animated.Stroke = "";
            animated.Stroke_width = 0;
            return animated;
        }

        public static string ExportBin(this g3.Polygon2d polygon, string targetFile = null)
        {
            targetFile = ValidateFileName(targetFile, ".poly");
            using (var writer = new BinaryWriter(new FileStream(targetFile, FileMode.OpenOrCreate)))
            {
                g3.gSerialization.Store(polygon, writer);
            }
            return targetFile;
        }

        public static string ExportBin(this IEnumerable<g3.Polygon2d> polygons, string targetFile = null)
        {
            string export = null;
            foreach (var polygon2d in polygons) export = polygon2d.ExportBin(targetFile);
            return export;
        }

        public static string ExportBin(this g3.GeneralPolygon2d polygon, string targetFile = null)
        {
            targetFile = ValidateFileName(targetFile, ".poly");
            using (var writer = new BinaryWriter(new FileStream(targetFile, FileMode.OpenOrCreate)))
            {
                g3.gSerialization.Store(polygon, writer);
            }
            return targetFile;
        }

        public static void ImportBin(this g3.Polygon2d poly, string sourceFile)
        {
            using (var reader = new BinaryReader(new FileStream(sourceFile, FileMode.Open, FileAccess.Read)))
            {
                g3.gSerialization.Restore(poly, reader);
            }
        }

        public static void ImportBin(this GeneralPolygon2d poly, string sourceFile)
        {
            using (var reader = new BinaryReader(new FileStream(sourceFile, FileMode.Open, FileAccess.Read)))
            {
                g3.gSerialization.Restore(poly, reader);
            }
        }
    }
}
