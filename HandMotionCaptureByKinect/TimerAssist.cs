using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;


namespace HandMotionCaptureByKinect
{
    public class TimerAssist
    {
        // 获得系统从启动到现在经过的毫秒数
        [DllImport("kernel32.dll")]
        public static extern uint GetTickCount();

        /// <summary>
        /// 计时器的关键时间点
        /// </summary>
        List<ulong> m_dwTimes = new List<ulong>();
        /// <summary>
        /// 对每一个关键点的描述
        /// </summary>
        List<string> m_descriptions = new List<string>();

        /// <summary>
        /// 开始计时
        /// </summary>
        public void Start()
        {
            AddTimePoint("开始");
        }

        /// <summary>
        /// 添加一个计时点
        /// </summary>
        /// <param name="description"></param>
        public void AddTimePoint(string description)
        {
            m_descriptions.Add(description);
            m_dwTimes.Add(GetTickCount());
        }

        /// <summary>
        /// 打印时间统计结果
        /// </summary>
        public void PrintResult()
        {
            for (int i = 1; i < m_dwTimes.Count; i++)			// 注意i从1开始，第一个时间点是计时的开始点
            {
                Console.WriteLine("{0}  耗时: {1} 毫秒\n", m_descriptions[i], m_dwTimes[i] - m_dwTimes[i - 1]);
            }

            Console.WriteLine("操作共耗时: {0} 毫秒\n", m_dwTimes[m_dwTimes.Count - 1] - m_dwTimes[0]);
        }

    }
}
