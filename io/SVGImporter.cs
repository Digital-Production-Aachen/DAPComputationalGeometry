using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml.Linq;

namespace g3
{
    public class SVGImporter
    {
        public readonly List<Styled<Polygon2d>> Polygons = new();
        public readonly List<Styled<Segment2d>> Lines = new();

        public Polygon2d[] Poly2dAsArray => Polygons.SelectToArray(x => x.Value);

        public static SVGImporter ImportSVG(string filePath)
        {
            var svgImporter = new SVGImporter();
            XDocument svgDoc = LoadSvgFromFile(filePath);

            foreach (var polygonElement in svgDoc.Descendants().Where(e => e.Name.LocalName == "polygon"))
            {
                string points = polygonElement.Attribute("points").Value;
                var vertices = points.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(p =>
                {
                    var coords = p.Split(',');
                    return new Vector2d(parseDouble(coords[0]), parseDouble(coords[1]));
                });

                Polygon2d polygon = SharedPolyPool.Rent(vertices);
                polygon.Reverse();

                var styledPoly = ExtractStyleAttributes(polygonElement, polygon);
                svgImporter.Polygons.Add(styledPoly);
            }

            foreach (var lineElement in svgDoc.Descendants().Where(e => e.Name.LocalName == "line"))
            {
                var x1 = parseDouble(lineElement.Attribute("x1").Value);
                var y1 = parseDouble(lineElement.Attribute("y1").Value);
                var x2 = parseDouble(lineElement.Attribute("x2").Value);
                var y2 = parseDouble(lineElement.Attribute("y2").Value);

                Segment2d line = new Segment2d(new Vector2d(x1, y1), new Vector2d(x2, y2));

                var styledLine = ExtractStyleAttributes(lineElement, line);
                svgImporter.Lines.Add(styledLine);
            }

            return svgImporter;
        }

        private static Styled<T> ExtractStyleAttributes<T>(XElement element, T target)
        {
            string style = element.Attribute("style")?.Value;
            var attributes = new Dictionary<string, string>();
            
            if (!string.IsNullOrEmpty(style))
            {
                var styleAttributes = style.Split(';');
                foreach (var attribute in styleAttributes)
                {
                    var keyValue = attribute.Split(':');
                    if (keyValue.Length == 2)
                    {
                        attributes[keyValue[0].Trim()] = keyValue[1].Trim();
                    }
                }
            }            

            return new Styled<T>(target, attributes);
        }

        private static double parseDouble(string input)
        {
            return double.Parse(input, System.Globalization.CultureInfo.InvariantCulture);
        }

        private static XDocument LoadSvgFromFile(string fileName)
        {
            string svgContent = File.ReadAllText(fileName);
            return XDocument.Parse(svgContent);
        }

        public class Styled<T>()
        {
            public readonly T Value;
            private Dictionary<string, string> _styleAttributes = new();

            public Styled(T value, IEnumerable<KeyValuePair<string, string>> styleAttributes) : this()
            {
                this.Value = value;
                foreach (var style in styleAttributes) { _styleAttributes[style.Key] = style.Value; }
            }

            public Styled(T value, Dictionary<string, string> styleAttributes) : this()
            {
                this.Value = value;
                _styleAttributes = styleAttributes;
            }

            public IReadOnlyDictionary<string, string> StyleAttributes => _styleAttributes;

            public string FillColor => TryGetStyleAttribute("fill");
            public string StrokeColor => TryGetStyleAttribute("stroke");
            public double StrokeWidth
            {
                get
                {
                    bool success = double.TryParse(TryGetStyleAttribute("stroke-width"), System.Globalization.CultureInfo.InvariantCulture, out double width);
                    return success ? width : 0;
                }
            }

            public string TryGetStyleAttribute(string attributeKey)
            {
                bool found = _styleAttributes.TryGetValue(attributeKey, out string value);
                return found ? value : string.Empty;
            }
        }
    }
}
