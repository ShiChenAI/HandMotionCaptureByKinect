using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HandMotionCaptureByKinect
{
    public enum HandPointType
    {
        /// <summary>
        /// 非手点
        /// </summary>
        None = 0,
        /// <summary>
        /// 左手伸直点
        /// </summary>
        LeftStraighten = 1,
        /// <summary>
        /// 左手弯曲
        /// </summary>
        LeftBend = 2,
        /// <summary>
        /// 右手伸直
        /// </summary>
        RightStraighten = 3,
        /// <summary>
        /// 右手弯曲
        /// </summary>
        RightBend = 4,
    }
}
