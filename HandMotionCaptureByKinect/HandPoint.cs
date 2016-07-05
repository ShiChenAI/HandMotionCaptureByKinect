using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HandMotionCaptureByKinect
{
    public class HandPoint
    {
        /// <summary>
        /// 像素点横坐标
        /// </summary>
        public int X = 0;
        /// <summary>
        /// 像素点纵坐标
        /// </summary>
        public int Y = 0;
        /// <summary>
        /// 像素点深度值
        /// </summary>
        public double D = 0;
        /// <summary>
        /// 像素点索引
        /// </summary>
        public int index = 0;
        /// <summary>
        /// 点的类型
        /// </summary>
        private HandPointType type = HandPointType.None;
        /// <summary>
        /// 读取手指点类型
        /// </summary>
        public HandPointType Type
        {
            get
            {
                return type;
            }
            set
            {
                type = value;
            }
        }
        /// <summary>
        /// 是否为轮廓点
        /// </summary>
        private bool bOutlinePoint = false;
        /// <summary>
        /// 读取是否为轮廓点判读值
        /// </summary>
        public bool IsOutlinePoint
        {
            get
            {
                return bOutlinePoint;
            }
            set
            {
                bOutlinePoint = value;
            }
        }
        /// <summary>
        /// 排序后的下一个点
        /// </summary>
        private HandPoint nextHandPoint = null;
        /// <summary>
        /// 读取排序后的下一个点
        /// </summary>
        public HandPoint NextHandPoint
        {
            get
            {
                return nextHandPoint;
            }
            set
            {
                nextHandPoint = value;
            }
        }
    }
}
