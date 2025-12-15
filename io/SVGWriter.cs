using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;

namespace g3
{
    // layer support ported from SutroMachine/geometry3Sharp by @gregmeess
    public class SVGWriter
    {
        public bool FlipY { get; set; } = true;

        public bool NonScalingStroke { get; set; } = false;

        protected class Layer
        {
            public Layer(string name)
            {
                Name = name;
                Objects = new List<object>();
            }

            public string Name { get; set; }
            public List<object> Objects { get; set; }
        }

        public class Style
        {
            public string Fill { get; set; }
            public string Stroke { get; set; }
            public float Stroke_width { get; set; }

            public float FillOpacity { get; set; } = 1;

            public static readonly Style Default = new Style() { Fill = "none", Stroke = "black", Stroke_width = 1 };

            public static Style Filled(string fillCol, string strokeCol = "", float strokeWidth = 0, float opacity = 1)
            {
                return new Style() { Fill = fillCol, FillOpacity = opacity, Stroke = strokeCol, Stroke_width = strokeWidth };
            }

            public static Style Outline(string strokeCol, float strokeWidth)
            {
                return new Style() { Fill = "none", Stroke = strokeCol, Stroke_width = strokeWidth };
            }

            public override string ToString()
            {
                StringBuilder b = new StringBuilder();

                if (Fill.Length > 0)
                {
                    b.Append("fill:");
                    b.Append(Fill);
                    b.Append(';');
                }

                if (FillOpacity < 1f)
                {
                    b.Append("fill-opacity:");
                    b.Append(FillOpacity);
                    b.Append(';');
                }

                if (Stroke.Length > 0)
                {
                    b.Append("stroke:");
                    b.Append(Stroke);
                    b.Append(';');
                }

                if (Stroke_width > 0)
                {
                    b.Append("stroke-width:");
                    b.Append(Stroke_width);
                    b.Append(";");
                }

                return b.ToString();
            }
        }

        protected Dictionary<object, Style> Styles = new Dictionary<object, Style>();

        public Style DefaultPolygonStyle { get; set; }
        public Style DefaultPolylineStyle { get; set; }
        public Style DefaultDGraphStyle { get; set; }
        public Style DefaultCircleStyle { get; set; }
        public Style DefaultArcStyle { get; set; }
        public Style DefaultLineStyle { get; set; }

        protected List<Layer> Layers;
        protected Layer CurrentLayer;

        protected AxisAlignedBox2d Bounds;

        public int Precision { get; set; } = 3;
        public double BoundsPad { get; set; } = 10;

        public SVGWriter()
        {
            CurrentLayer = new Layer("default");
            Layers = new List<Layer>() { CurrentLayer };

            Bounds = AxisAlignedBox2d.Empty;

            DefaultPolygonStyle = Style.Outline("grey", 1);
            DefaultPolylineStyle = Style.Outline("cyan", 1);
            DefaultCircleStyle = Style.Filled("green", "black", 1);
            DefaultArcStyle = Style.Outline("magenta", 1);
            DefaultLineStyle = Style.Outline("black", 1);
            DefaultDGraphStyle = Style.Outline("blue", 1);
        }

        public virtual void SetDefaultLineWidth(float width)
        {
            DefaultPolygonStyle.Stroke_width = width;
            DefaultPolylineStyle.Stroke_width = width;
            DefaultCircleStyle.Stroke_width = width;
            DefaultArcStyle.Stroke_width = width;
            DefaultLineStyle.Stroke_width = width;
            DefaultDGraphStyle.Stroke_width = width;
        }

        public virtual void StartNewLayer(string name)
        {
            CurrentLayer = new Layer(name);
            Layers.Add(CurrentLayer);
        }

        public virtual void AddPolygon(Polygon2d poly)
        {
            CurrentLayer.Objects.Add(poly);
            Bounds.Contain(poly.Bounds);
        }

        public virtual void AddPolygon(Polygon2d poly, Style style)
        {
            CurrentLayer.Objects.Add(poly);
            Styles[poly] = style;
            Bounds.Contain(poly.Bounds);
        }

        public virtual void AddPolygon(GeneralPolygon2d poly)
        {
            CurrentLayer.Objects.Add(poly);
            Bounds.Contain(poly.Bounds);
        }

        public virtual void AddPolygon(GeneralPolygon2d poly, Style style)
        {
            CurrentLayer.Objects.Add(poly);
            Styles[poly] = style;
            Bounds.Contain(poly.Bounds);
        }

        public virtual void AddBox(AxisAlignedBox2d box)
        {
            AddBox(box, DefaultPolygonStyle);
        }

        public virtual void AddBox(AxisAlignedBox2d box, Style style)
        {
            Polygon2d poly = new Polygon2d();
            for (int k = 0; k < 4; ++k)
                poly.AppendVertex(box.GetCorner(k));
            AddPolygon(poly, style);
        }

        public virtual void AddPolyline(PolyLine2d poly)
        {
            CurrentLayer.Objects.Add(poly);
            Bounds.Contain(poly.Bounds);
        }

        public virtual void AddPolyline(PolyLine2d poly, Style style)
        {
            CurrentLayer.Objects.Add(poly);
            Styles[poly] = style;
            Bounds.Contain(poly.Bounds);
        }

        public virtual void AddGraph(DGraph2 graph)
        {
            CurrentLayer.Objects.Add(graph);
            Bounds.Contain(graph.GetBounds());
        }

        public virtual void AddGraph(DGraph2 graph, Style style)
        {
            CurrentLayer.Objects.Add(graph);
            Styles[graph] = style;
            Bounds.Contain(graph.GetBounds());
        }

        public virtual void AddCircle(Circle2d circle)
        {
            CurrentLayer.Objects.Add(circle);
            Bounds.Contain(circle.Bounds);
        }

        public virtual void AddCircle(Circle2d circle, Style style)
        {
            CurrentLayer.Objects.Add(circle);
            Styles[circle] = style;
            Bounds.Contain(circle.Bounds);
        }

        public virtual void AddArc(Arc2d arc)
        {
            CurrentLayer.Objects.Add(arc);
            Bounds.Contain(arc.Bounds);
        }

        public virtual void AddArc(Arc2d arc, Style style)
        {
            CurrentLayer.Objects.Add(arc);
            Styles[arc] = style;
            Bounds.Contain(arc.Bounds);
        }

        public virtual void AddLine(Segment2d segment)
        {
            CurrentLayer.Objects.Add(new Segment2dBox(segment));
            Bounds.Contain(segment.P0); Bounds.Contain(segment.P1);
        }

        public virtual void AddLine(Segment2d segment, Style style)
        {
            Segment2dBox segbox = new Segment2dBox(segment);
            CurrentLayer.Objects.Add(segbox);
            Styles[segbox] = style;
            Bounds.Contain(segment.P0); Bounds.Contain(segment.P1);
        }

        public virtual void AddComplex(PlanarComplex complex)
        {
            CurrentLayer.Objects.Add(complex);
            Bounds.Contain(complex.Bounds());
        }

        public virtual IOWriteResult Write(string sFilename)
        {
            var current_culture = Thread.CurrentThread.CurrentCulture;

            try
            {
                // push invariant culture for write
                Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;

                using (StreamWriter w = new StreamWriter(sFilename))
                {
                    if (w.BaseStream == null)
                        return new IOWriteResult(IOCode.FileAccessError, "Could not open file " + sFilename + " for writing");

                    write_header_1_1(w);

                    foreach (var layer in Layers)
                    {
                        if (layer.Objects.Count == 0)
                            continue;

                        OpenGroup(layer.Name, w);
                        WriteObjects(layer.Objects, w);
                        CloseGroup(w);
                    }
                    w.WriteLine("</svg>");
                }

                // restore culture
                Thread.CurrentThread.CurrentCulture = current_culture;
                return IOWriteResult.Ok;
            }
            catch (Exception e)
            {
                Thread.CurrentThread.CurrentCulture = current_culture;
                return new IOWriteResult(IOCode.WriterError, "Unknown error : exception : " + e.Message);
            }
        }

        protected virtual void CloseGroup(StreamWriter w)
        {
            w.WriteLine("</g>");
        }

        protected virtual void OpenGroup(string name, StreamWriter w)
        {
            w.WriteLine($"<g id=\"{name}\">");
        }

        protected virtual void WriteObjects(List<object> objects, StreamWriter w)
        {
            foreach (var o in objects)
            {
                switch (o)
                {
                    case Polygon2d polygon:
                        write_polygon(polygon, w);
                        break;

                    case GeneralPolygon2d gpolygon:
                        write_general_polygon(gpolygon, w);
                        break;

                    case PolyLine2d polyline:
                        write_polyline(polyline, w);
                        break;

                    case Circle2d circle:
                        write_circle(circle, w);
                        break;

                    case Arc2d arc:
                        write_arc(arc, w);
                        break;

                    case Segment2dBox segmentBox:
                        write_line(segmentBox, w);
                        break;

                    case DGraph2 graph:
                        write_graph(graph, w);
                        break;

                    case PlanarComplex complex:
                        write_complex(complex, w);
                        break;

                    default:
                        throw new NotSupportedException("SVGWriter.Write: unknown object type " + o.GetType().ToString());
                }
            }
        }

        public static void QuickWrite(List<GeneralPolygon2d> polygons, string sPath, double line_width = 1)
        {
            SVGWriter writer = new SVGWriter();
            Style outer_cw = SVGWriter.Style.Outline("black", 2 * (float)line_width);
            Style outer_ccw = SVGWriter.Style.Outline("green", 2 * (float)line_width);
            Style inner = SVGWriter.Style.Outline("red", (float)line_width);
            foreach (GeneralPolygon2d poly in polygons)
            {
                if (poly.Outer.IsClockwise)
                    writer.AddPolygon(poly.Outer, outer_cw);
                else
                    writer.AddPolygon(poly.Outer, outer_ccw);
                foreach (var hole in poly.Holes)
                    writer.AddPolygon(hole, inner);
            }
            writer.Write(sPath);
        }

        public static void QuickWrite(DGraph2 graph, string sPath, double line_width = 1)
        {
            SVGWriter writer = new SVGWriter();
            Style style = SVGWriter.Style.Outline("black", (float)line_width);
            writer.AddGraph(graph, style);
            writer.Write(sPath);
        }

        public static void QuickWrite(List<GeneralPolygon2d> polygons1, string color1, float width1,
                                      List<GeneralPolygon2d> polygons2, string color2, float width2,
                                      string sPath)
        {
            SVGWriter writer = new SVGWriter();
            Style style1 = SVGWriter.Style.Outline(color1, width1);
            Style style1_holes = SVGWriter.Style.Outline(color1, width1 / 2);
            foreach (GeneralPolygon2d poly in polygons1)
            {
                writer.AddPolygon(poly.Outer, style1);
                foreach (var hole in poly.Holes)
                    writer.AddPolygon(hole, style1_holes);
            }
            Style style2 = SVGWriter.Style.Outline(color2, width2);
            Style style2_holes = SVGWriter.Style.Outline(color2, width2 / 2);
            foreach (GeneralPolygon2d poly in polygons2)
            {
                writer.AddPolygon(poly.Outer, style2);
                foreach (var hole in poly.Holes)
                    writer.AddPolygon(hole, style2_holes);
            }
            writer.Write(sPath);
        }

        protected virtual Vector2d MapPt(Vector2d v)
        {
            if (FlipY)
            {
                return new Vector2d(v.x, Bounds.Min.y + (Bounds.Max.y - v.y));
            }
            else
                return v;
        }

        protected virtual void write_header_1_1(StreamWriter w)
        {
            StringBuilder b = new StringBuilder();

            b.Append("<svg ");
            b.Append("version=\"1.1\" ");
            b.Append("xmlns=\"http://www.w3.org/2000/svg\" ");
            b.Append("xmlns:xlink=\"http://www.w3.org/1999/xlink\" ");
            b.Append("x=\"0px\" y=\"0px\" ");
            b.Append(string.Format("viewBox=\"{0} {1} {2} {3}\" ",
                                   Math.Round(Bounds.Min.x - BoundsPad, Precision),
                                   Math.Round(Bounds.Min.y - BoundsPad, Precision),
                                   Math.Round(Bounds.Width + 2 * BoundsPad, Precision),
                                   Math.Round(Bounds.Height + 2 * BoundsPad, Precision)));
            b.Append('>');

            w.WriteLine(b);
        }

        protected virtual void write_polygon(Polygon2d poly, StreamWriter w)
        {
            StringBuilder b = new StringBuilder();
            b.Append("<polygon points=\"");
            for (int i = 0; i < poly.VertexCount; ++i)
            {
                Vector2d v = MapPt(poly[i]);
                b.Append(Math.Round(v.x, Precision));
                b.Append(',');
                b.Append(Math.Round(v.y, Precision));
                if (i < poly.VertexCount - 1)
                    b.Append(' ');
            }
            b.Append("\" ");
            append_style(b, poly, DefaultPolygonStyle);
            b.Append(" />");

            w.WriteLine(b);
        }

        protected virtual void write_general_polygon(GeneralPolygon2d poly, StreamWriter w)
        {
            StringBuilder b = new StringBuilder();
            b.Append("<path d=\"");
            AppendPath(b, poly.Outer);
            foreach (var hole in poly.Holes)
            {
                AppendPath(b, hole);
            }

            b.Append("\" ");
            append_style(b, poly, DefaultPolygonStyle);
            b.Append(" />");

            w.WriteLine(b);
        }

        protected virtual void write_polyline(PolyLine2d poly, StreamWriter w)
        {
            StringBuilder b = new StringBuilder();
            b.Append("<polyline points=\"");
            for (int i = 0; i < poly.VertexCount; ++i)
            {
                Vector2d v = MapPt(poly[i]);
                b.Append(Math.Round(v.x, Precision));
                b.Append(',');
                b.Append(Math.Round(v.y, Precision));
                if (i < poly.VertexCount - 1)
                    b.Append(' ');
            }
            b.Append("\" ");
            append_style(b, poly, DefaultPolylineStyle);
            b.Append(" />");

            w.WriteLine(b);
        }

        protected virtual void write_graph(DGraph2 graph, StreamWriter w)
        {
            string style = get_style(graph, DefaultDGraphStyle);

            StringBuilder b = new StringBuilder();
            foreach (int eid in graph.EdgeIndices())
            {
                Segment2d seg = graph.GetEdgeSegment(eid);
                b.Append("<line ");
                Vector2d p0 = MapPt(seg.P0), p1 = MapPt(seg.P1);
                append_property("x1", p0.x, b, true);
                append_property("y1", p0.y, b, true);
                append_property("x2", p1.x, b, true);
                append_property("y2", p1.y, b, true);
                b.Append(style);
                b.Append(" />");
                b.AppendLine();
            }
            w.WriteLine(b);
        }

        protected virtual void write_circle(Circle2d circle, StreamWriter w)
        {
            StringBuilder b = new StringBuilder();
            b.Append("<circle ");
            Vector2d c = MapPt(circle.Center);
            append_property("cx", c.x, b, true);
            append_property("cy", c.y, b, true);
            append_property("r", circle.Radius, b, true);
            append_style(b, circle, DefaultCircleStyle);
            b.Append(" />");
            w.WriteLine(b);
        }

        protected virtual void write_arc(Arc2d arc, StreamWriter w)
        {
            StringBuilder b = new StringBuilder();
            Vector2d vStart = MapPt(arc.P0);
            Vector2d vEnd = MapPt(arc.P1);
            b.Append("<path ");
            b.Append("d=\"");

            // move to start coordinates
            b.Append("M");
            b.Append(Math.Round(vStart.x, Precision));
            b.Append(",");
            b.Append(Math.Round(vStart.y, Precision));
            b.Append(" ");

            // start arc
            b.Append("A");

            // radii (write twice because this is actually elliptical arc)
            b.Append(Math.Round(arc.Radius, Precision));
            b.Append(",");
            b.Append(Math.Round(arc.Radius, Precision));
            b.Append(" ");

            b.Append("0 ");     // x-axis-rotation

            int large = (arc.AngleEndDeg - arc.AngleStartDeg) > 180 ? 1 : 0;
            int sweep = (arc.IsReversed) ? 1 : 0;
            b.Append(large);
            b.Append(",");
            b.Append(sweep);

            // end coordinates
            b.Append(Math.Round(vEnd.x, Precision));
            b.Append(",");
            b.Append(Math.Round(vEnd.y, Precision));

            b.Append("\" ");     // close path

            append_style(b, arc, DefaultArcStyle);

            b.Append(" />");
            w.WriteLine(b);
        }

        protected virtual void write_line(Segment2dBox segbox, StreamWriter w)
        {
            Segment2d seg = (Segment2d)segbox;
            StringBuilder b = new StringBuilder();
            b.Append("<line ");
            Vector2d p0 = MapPt(seg.P0), p1 = MapPt(seg.P1);
            append_property("x1", p0.x, b, true);
            append_property("y1", p0.y, b, true);
            append_property("x2", p1.x, b, true);
            append_property("y2", p1.y, b, true);
            append_style(b, segbox, DefaultLineStyle);
            b.Append(" />");
            w.WriteLine(b);
        }

        protected virtual void write_complex(PlanarComplex complex, StreamWriter w)
        {
            foreach (var elem in complex.ElementsItr())
            {
                List<IParametricCurve2d> curves = CurveUtils2.Flatten(elem.source);
                foreach (IParametricCurve2d c in curves)
                {
                    if (c is Segment2d)
                        write_line(new Segment2dBox((Segment2d)c), w);
                    else if (c is Circle2d)
                        write_circle(c as Circle2d, w);
                    else if (c is Polygon2DCurve)
                        write_polygon((c as Polygon2DCurve).Polygon, w);
                    else if (c is PolyLine2DCurve)
                        write_polyline((c as PolyLine2DCurve).Polyline, w);
                    else if (c is Arc2d)
                        write_arc(c as Arc2d, w);
                }
            }
        }

        protected virtual void append_property(string name, double val, StringBuilder b, bool trailSpace = true)
        {
            b.Append(name); b.Append("=\"");
            b.Append(Math.Round(val, Precision));
            if (trailSpace)
                b.Append("\" ");
            else
                b.Append("\"");
        }

        protected virtual void append_style(StringBuilder b, object o, Style defaultStyle)
        {
            Style style;
            if (!Styles.TryGetValue(o, out style))
                style = defaultStyle;
            b.Append("style=\"");
            b.Append(style.ToString());
            b.Append("\"");
        }

        protected virtual void AppendPath(StringBuilder b, Polygon2d poly)
        {
            b.Append("M ");
            for (int i = 0; i < poly.VertexCount; ++i)
            {
                Vector2d v = MapPt(poly[i]);
                b.Append(Math.Round(v.x, Precision));
                b.Append(',');
                b.Append(Math.Round(v.y, Precision));
                b.Append(' ');
            }
            b.Append("z");
        }

        protected virtual string get_style(object o, Style defaultStyle)
        {
            Style style;
            if (!Styles.TryGetValue(o, out style))
                style = defaultStyle;
            return "style=\"" + style.ToString() + "\"";
        }
    }
}