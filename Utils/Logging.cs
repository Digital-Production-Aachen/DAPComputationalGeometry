using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace g3
{
    public interface ILogging
    {
        /// <summary>
        /// Logging singleton instance
        /// </summary>
        public static ILogging Logger { get; set; } = null;
        public void Error(string message);
        public void Error(Exception e, string message);
        public void Warning(string message);
        public void Warning(Exception e, string message);
        public void Information(string message);
        public void Information(Exception e, string message);
    }
}
