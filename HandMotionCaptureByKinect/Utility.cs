using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HandMotionCaptureByKinect
{
    public class Utility
    {
        /// <summary>
        /// 像素点数组转换为屏幕二维数组
        /// </summary>
        /// <param name="pixelArray">像素点数组</param>
        /// <param name="screenArray">屏幕二维数组</param>
        public static void PixelArrayToScreenArray(uint[] pixelArray, ref uint[ , ] screenArray)
        {
            for (int i = 0; i < pixelArray.Length; i++)
            {
                int x = i / 512;
                int y = i % 512;
                screenArray[x, y] = pixelArray[i];
            }
        }

        /// <summary>
        /// 屏幕二维数组转换为像素点数组
        /// </summary>
        /// <param name="screenArray">屏幕二维数组</param>
        /// <param name="pixelArray">像素点数组</param>
        public static void ScreenArrayToPixelArray(uint[,] screenArray, ref uint[] pixelArray)
        {
            for (int i = 0; i < screenArray.GetLength(0); i++)
            {
                for (int j = 0; j < screenArray.GetLength(1); j++)
                {
                    int index = i * 512 + j;
                    pixelArray[index] = screenArray[i, j];
                }
            }
        }

        /// <summary>
        /// 像素点转换为屏幕二维点
        /// </summary>
        /// <param name="pixelIndex">像素点</param>
        /// <param name="screenPointX">屏幕二维点横坐标</param>
        /// <param name="screenPointY">屏幕二维点纵坐标</param>
        public static void PixelIndexToScreenPoint(int pixelIndex, ref int screenPointX, ref int screenPointY)
        {
            screenPointX = pixelIndex / 512;
            screenPointY = pixelIndex % 512;
        }

        /// <summary>
        /// 屏幕二维点转换为像素点
        /// </summary>
        /// <param name="screenPointX">屏幕二维点横坐标</param>
        /// <param name="screenPointY">屏幕二维点纵坐标</param>
        /// <param name="pixelIndex">像素点</param>
        public static void ScreenPointPixelIndex(int screenPointX, int screenPointY, ref int pixelIndex)
        {
            pixelIndex = screenPointX * 512 + screenPointY;
        }

        /// <summary>
        /// 判断两个浮点数是否相等
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="tol"></param>
        /// <returns></returns>
        public static bool IsEqual(float a, float b, double tol = 1.0e-5)
        {
            return (Math.Abs(a - b) < tol);
        }
    }
}
