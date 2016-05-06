using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.IO;
using System.Data;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.Util;

namespace HandMotionCaptureByKinect
{
    /// <summary>
    /// PCAWindow.xaml 的交互逻辑
    /// </summary>
    public partial class PCAWindow : Window
    {
        private Matrix<double> m_inputMatrix;

        public PCAWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            
        }

        private void btnRead_Click(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.OpenFileDialog ofd = new Microsoft.Win32.OpenFileDialog();
            ofd.DefaultExt = ".txt";
            ofd.Filter = "txt file|*.txt";
            if (ofd.ShowDialog() == true)
            {
                StreamReader file = new StreamReader(@ofd.FileName);
                string line = "";
                List<string[]> inputList = new List<string[]>();
                
                while ((line = file.ReadLine()) != null)
                {
                    string[] strValue = line.Split(' ');
                    inputList.Add(strValue);
                }
                file.Close();

                m_inputMatrix = new Matrix<double>(inputList.Count, 14);

                for (int i = 0; i < inputList.Count; i++)
                {
                    string[] str = inputList[i];
                    for (int j = 0; j < str.Length; j++)
                    {
                        double value = Convert.ToDouble(str[j]);
                        m_inputMatrix.Data[i, j] = value;
                    }
                }

                gridInput.Items.Clear();
                DataTable dt = new DataTable();
                dt.Columns.Add(new DataColumn("Column1"));
                dt.Columns.Add(new DataColumn("Column2"));
                dt.Columns.Add(new DataColumn("Column3"));
                dt.Columns.Add(new DataColumn("Column4"));
                dt.Columns.Add(new DataColumn("Column5"));
                dt.Columns.Add(new DataColumn("Column6"));
                dt.Columns.Add(new DataColumn("Column7"));
                dt.Columns.Add(new DataColumn("Column8"));
                dt.Columns.Add(new DataColumn("Column9"));
                dt.Columns.Add(new DataColumn("Column10"));
                dt.Columns.Add(new DataColumn("Column11"));
                dt.Columns.Add(new DataColumn("Column12"));
                dt.Columns.Add(new DataColumn("Column13"));
                dt.Columns.Add(new DataColumn("Column14"));

                DataRow dr;
                for (int i = 0; i < inputList.Count; i++)
                {
                    dr = dt.NewRow();
                    for (int j = 0; j < 14; j++)
                    {
                        dr[j] = m_inputMatrix.Data[i, j];
                        dt.Rows.Add(dr);
                    }
                }

                gridInput.ItemsSource = dt.DefaultView;
            }
        }

        private void btnCalculate_Click(object sender, RoutedEventArgs e)
        {

        }
    }
}
