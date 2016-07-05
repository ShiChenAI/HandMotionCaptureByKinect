using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;

namespace HandMotionCaptureByKinect
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private const int BytesPerPixel = 4;
        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;
        /// <summary>
        /// Collection of colors to be used to display the BodyIndexFrame data.
        /// </summary>
        private static readonly uint[] BodyColor =
        {
            0x0000FF00,
            0x00FF0000,
            0xFFFF4000,
            0x40FFFF00,
            0xFF40FF00,
            0xFF808000,
            //0x0000FF00,
            //0x0000FF00,
            //0x0000FF00,
            //0x0000FF00,
            //0x0000FF00,
        };
        /// <summary>
        /// 左手外围轮廓颜色
        /// </summary>
        private static readonly uint leftHandOutlineColor = 0x00FF0000;     //  红
        /// <summary>
        /// 左手弯曲手指轮廓颜色
        /// </summary>
        private static readonly uint leftBendHandOutlineColor = 0x00FFFF00;     // 黄
        /// <summary>
        /// 右手外围轮廓颜色
        /// </summary>
        private static readonly uint rightHandOutlineColor = 0x000000FF;    // 蓝
        /// <summary>
        /// 右手弯曲手指轮廓颜色
        /// </summary>
        private static readonly uint rightBendHandOutlineColor = 0x00008000;    // 绿
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;
        /// <summary>
        /// Reader for depth/color/body index frames
        /// </summary>
        private MultiSourceFrameReader reader;
        /// <summary>
        /// Description of the data contained in the body index frame
        /// </summary>
        private FrameDescription bodyIndexFrameDescription = null;
        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap bodyIndexBitmap = null;
        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private uint[] bodyIndexPixels = null;
        /// <summary>
        /// The coordinate mapper to convert between depth and color frames of reference
        /// </summary>
        private CoordinateMapper mapper;
        /// <summary>
        /// Image Width of depth frame
        /// </summary>
        private int depthWidth = 0;
        /// <summary>
        /// Image height of depth frame
        /// </summary>
        private int depthHeight = 0;
        /// <summary>
        /// Count of pixels in the depth frame
        /// </summary>
        private int depthPixelCount = 0;
        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;
        /// <summary>
        /// 手部范围阈值
        /// </summary>
        private float maxHandDist = 0;
        /// <summary>
        /// 右手手腕深度值
        /// </summary>
        private float rightWristDist = 0;
        /// <summary>
        /// 左手手腕深度值
        /// </summary>
        private float leftWristDist = 0;
        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;
        /// <summary>
        /// 保存手的信息(0-其他,1-左手, 2-左手弯曲, 3-右手, 4-右手弯曲)
        /// </summary>
        private int[] handPixels = null;
        /// <summary>
        /// 深度信息
        /// </summary>
        private FrameDescription depthFrameDescription = null;
        /// <summary>
        /// 最远的手(1-左, 2-右)
        /// </summary>
        private int maxDistHand = 0;
        /// <summary>
        /// 手的像素点限制区域宽度
        /// </summary>
        private int handPixelLimitedWidth = 50;
        /// <summary>
        /// 手的像素点限制区域高度
        /// </summary>
        private int handPixelLimitedHeight = 50;
        /// <summary>
        /// 左手中心点的像素点
        /// </summary>
        private int leftHandCenterPixel = 0;
        /// <summary>
        /// 右手中心点的像素点
        /// </summary>
        private int rightHandCenterPixel = 0;
        /// <summary>
        /// 左手中心点的像素点横坐标
        /// </summary>
        private int leftHandCenterPixelX = 0;
        /// <summary>
        /// 左手中心点的像素点纵坐标
        /// </summary>
        private int leftHandCenterPixelY = 0;
        /// <summary>
        /// 右手中心点的像素点横坐标
        /// </summary>
        private int rightHandCenterPixelX = 0;
        /// <summary>
        /// 右手中心点的像素点纵坐标
        /// </summary>
        private int rightHandCenterPixelY = 0;
        /// <summary>
        /// 弯曲手指距离阈值
        /// </summary>
        private double bendFigerhreshold = 60;
        /// <summary>
        /// 左手伸直点集合
        /// </summary>
        private List<HandPoint> leftHandStraightenPts = null;
        /// <summary>
        /// 左手弯曲点集合
        /// </summary>
        private List<HandPoint> leftHandBendPts = null;
        /// <summary>
        /// 右手伸直点集合
        /// </summary>
        private List<HandPoint> rightHandStraightenPts = null;
        /// <summary>
        /// 右手弯曲点集合
        /// </summary>
        private List<HandPoint> rightHandBendPts = null;

        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.mapper = this.kinectSensor.CoordinateMapper;

            this.reader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.BodyIndex | FrameSourceTypes.Body);

            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.depthWidth = depthFrameDescription.Width;
            this.depthHeight = depthFrameDescription.Height;
            this.depthPixelCount = this.depthWidth * this.depthHeight;

            this.bodyIndexFrameDescription = this.kinectSensor.BodyIndexFrameSource.FrameDescription;

            // Add an event handler to be called whenever depth and color both have new data
            this.reader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            // allocate space to put the pixels being converted
            this.bodyIndexPixels = new uint[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // create the bitmap to display
            this.bodyIndexBitmap = new WriteableBitmap(this.bodyIndexFrameDescription.Width, this.bodyIndexFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            

            // set IsAvailableChanged event notifier
            //this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {

        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            this.reader.MultiSourceFrameArrived -= this.Reader_MultiSourceFrameArrived;

            if (this.reader != null)
            {
                this.reader.Dispose();
                this.reader = null;
            }

            if (null != this.kinectSensor)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.bodyIndexBitmap;
            }
        }

        /// <summary>
        /// Event handler for multiSourceFrame arrived event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            ProcessMultiFrameByHandDepth(e);
        }


        /// <summary>
        /// 通过手的深度数据计算
        /// </summary>
        /// <param name="e"></param>
        private void ProcessMultiFrameByHandDepth(MultiSourceFrameArrivedEventArgs e)
        {
            // 初始化
            MultiSourceFrameReference multiFrameReference = e.FrameReference;
            MultiSourceFrame multiSourceFrame = multiFrameReference.AcquireFrame();

            // 获取骨骼信息
            TimerAssist ta = new TimerAssist();
            ta.Start();
            bool dataReceived = false;
            BodyFrameReference bodyFrameReference = multiSourceFrame.BodyFrameReference;
            using (BodyFrame bodyFrame = bodyFrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (!dataReceived)
            {
                return;
            }

            //             if (dataReceived)
            //             {
            //                 foreach (Body body in this.bodies)
            //                 {
            //                     if (body.IsTracked)
            //                     {
            //                         IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
            // 
            //                         // 获取手腕坐标
            //                         float rightHandDist = joints[JointType.WristRight].Position.Z;
            //                         float leftHandDist = joints[JointType.WristLeft].Position.Z;
            //                         //float rightHandDist = joints[JointType.ElbowRight].Position.Z;
            //                         //float leftHandDist = joints[JointType.ElbowLeft].Position.Z;
            // 
            // 
            //                         // 计算手部捕捉阈值
            //                         maxHandDist = rightHandDist > leftHandDist ? rightHandDist * 1000 : leftHandDist * 1000;
            //                     }
            //                 }
            //             }

            foreach (Body body in this.bodies)
            {
                if (body.IsTracked)
                {
                    IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                    // 获取手腕坐标
                    rightWristDist = joints[JointType.WristRight].Position.Z * 1000;
                    leftWristDist = joints[JointType.WristLeft].Position.Z * 1000;
                    
                    //float rightHandDist = joints[JointType.ElbowRight].Position.Z;
                    //float leftHandDist = joints[JointType.ElbowLeft].Position.Z;

                    //                     // 获取手掌坐标
                    //                     float rightHandDist = joints[JointType.HandRight].Position.Z;
                    //                     float leftHandDist = joints[JointType.HandLeft].Position.Z;
                    // 
                    //                     // 计算深度阈值范围
                    //                     float rightGap = Math.Abs(rightWristDist - rightHandDist);
                    //                     float leftGap = Math.Abs(leftWristDist - leftHandDist);
                    // 
                    //                     this.rightMiniDist = (rightHandDist - rightGap) * 1000;
                    //                     this.rightMaxDist = rightWristDist * 1000;
                    // 
                    //                     this.leftMiniDist = (leftHandDist - leftGap) * 1000;
                    //                     this.leftMaxDist = leftWristDist * 1000;

                    // 计算手部捕捉阈值
                    maxHandDist = rightWristDist > leftWristDist ? rightWristDist : leftWristDist;

                    // 获取左右手的中心像素点
                    CameraSpacePoint position = joints[JointType.HandLeft].Position;
                    DepthSpacePoint depthSpacePoint = this.mapper.MapCameraPointToDepthSpace(position);
                    Utility.ScreenPointPixelIndex((int)depthSpacePoint.X, (int)depthSpacePoint.Y, ref leftHandCenterPixel);
                    leftHandCenterPixelX = (int)depthSpacePoint.X;
                    leftHandCenterPixelY = (int)depthSpacePoint.Y;


                    position = joints[JointType.HandRight].Position;
                    depthSpacePoint = this.mapper.MapCameraPointToDepthSpace(position);
                    Utility.ScreenPointPixelIndex((int)depthSpacePoint.X, (int)depthSpacePoint.Y, ref rightHandCenterPixel); 
                    rightHandCenterPixelX = (int)depthSpacePoint.X;
                    rightHandCenterPixelY = (int)depthSpacePoint.Y;

                    double handDist = joints[JointType.HandLeft].Position.Z;
                    double wristDist = joints[JointType.WristLeft].Position.Z;
                }
            }

            //             if (this.rightMiniDist == 0 || this.rightMaxDist == 0 || this.leftMiniDist == 0 || this.leftMaxDist == 0)
            //             {
            //                 return;
            //             }

            if (maxHandDist == 0)
            {
                return;
            }

            if (Utility.IsEqual(leftWristDist, maxHandDist))
            {
                maxDistHand = 1;
            }
            else if (Utility.IsEqual(rightWristDist, maxHandDist))
            {
                maxDistHand = 2;
            }

            ta.AddTimePoint("骨骼信息");

            // 获取深度信息
            bool depthFrameProcessed = false;
            DepthFrameReference depthFrameReference = multiSourceFrame.DepthFrameReference;
            using (DepthFrame depthFrame = depthFrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.bodyIndexBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.bodyIndexBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //// maxDepth = depthFrame.DepthMaxReliableDistance

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (!depthFrameProcessed)
            {
                return;
            }

            ta.AddTimePoint("深度信息");

            // 获取身体指数信息
            bool bodyIndexFrameProcessed = false;
            BodyIndexFrameReference bodyIndexFrameReference = multiSourceFrame.BodyIndexFrameReference;
            using (BodyIndexFrame bodyIndexFrame = bodyIndexFrameReference.AcquireFrame())
            {
                if (bodyIndexFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer bodyIndexBuffer = bodyIndexFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height) == bodyIndexBuffer.Size) &&
                            (this.bodyIndexFrameDescription.Width == this.bodyIndexBitmap.PixelWidth) && (this.bodyIndexFrameDescription.Height == this.bodyIndexBitmap.PixelHeight))
                        {
                            // 遍历像素点集合, 将大于手部阈值的点置为255
                            this.ProcessBodyIndexFrameData(bodyIndexBuffer.UnderlyingBuffer, bodyIndexBuffer.Size);
                            bodyIndexFrameProcessed = true;
                        }
                    }
                }
            }
            ta.AddTimePoint("身体指数信息");

            if (bodyIndexFrameProcessed)
            {
                this.RenderBodyIndexPixels();
            }

            ta.AddTimePoint("绘制");
            ta.PrintResult();
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the BodyIndexFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the bodyIndexFrameData pointer.
        /// </summary>
        /// <param name="bodyIndexFrameData">Pointer to the BodyIndexFrame image data</param>
        /// <param name="bodyIndexFrameDataSize">Size of the BodyIndexFrame image data</param>
        private unsafe void ProcessBodyIndexFrameData(IntPtr bodyIndexFrameData, uint bodyIndexFrameDataSize)
        {
            // 计算左右手限制范围
            int leftHandPixelXMin = leftHandCenterPixelX - handPixelLimitedWidth;
            int leftHandPixelXMax = leftHandCenterPixelX + handPixelLimitedWidth;
            int leftHandPixelYMin = leftHandCenterPixelY - handPixelLimitedHeight;
            int leftHandPixelYMax = leftHandCenterPixelY + handPixelLimitedHeight;

            int rightHandPixelXMin = rightHandCenterPixelX - handPixelLimitedWidth;
            int rightHandPixelXMax = rightHandCenterPixelX + handPixelLimitedWidth;
            int rightHandPixelYMin = rightHandCenterPixelY - handPixelLimitedHeight;
            int rightHandPixelYMax = rightHandCenterPixelY + handPixelLimitedHeight;

            // 初始化轮廓点数组
            leftHandStraightenPts = new List<HandPoint>();
            leftHandBendPts = new List<HandPoint>();
            rightHandStraightenPts = new List<HandPoint>();
            rightHandBendPts = new List<HandPoint>();

            // 用来遍历的数组
            List<HandPoint> tempLeftHandStraightenPts = new List<HandPoint>();
            List<HandPoint> tempLeftHandBendPts = new List<HandPoint>();
            List<HandPoint> tempRightHandStraightenPts = new List<HandPoint>();
            List<HandPoint> tempRightHandBendPts = new List<HandPoint>();

            // 获取bodyIndex数组
            byte* frameData = (byte*)bodyIndexFrameData;

            // 轮廓点判断
            int startIndex = 0;
            int limitIndex = 0;
            Utility.ScreenPointPixelIndex(leftHandPixelXMin, leftHandPixelYMin, ref limitIndex);
            for (int i = limitIndex; i < this.handPixels.Count(); i++)
            {
                // 获取第一个点
                if (frameData[i] < BodyColor.Length && this.depthPixels[i] != 0)
                {
                    if (handPixels[i] == 1)
                    {
                        if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
                        {
                            startIndex = i;
                            this.bodyIndexPixels[i] = leftBendHandOutlineColor;
                            break;
                        }
                    }
                    //if (frameData[i] < BodyColor.Length && this.depthPixels[i] != 0)
                    //{
                    //    this.bodyIndexPixels[i] = 0x00000000;
                    //    int pixelX = 0, pixelY = 0;
                    //    Utility.PixelIndexToScreenPoint(i, ref pixelX, ref pixelY);

                    //    // 初始化像素点
                    //    HandPoint hp = new HandPoint();
                    //    hp.X = pixelX;
                    //    hp.Y = pixelY;
                    //    hp.D = this.depthPixels[i];
                    //    hp.index = i;

                    //    switch (handPixels[i])
                    //    {
                    //        case 1:         // 左手
                    //                        // 进行外部点的判断                  
                    //            if (radShowAll.IsChecked == true)
                    //            {
                    //                // 进行范围判定
                    //                if (pixelX <= leftHandPixelXMax && pixelX >= leftHandPixelXMin && pixelY <= leftHandPixelYMax && pixelY >= leftHandPixelYMin)
                    //                {
                    //                    this.bodyIndexPixels[i] = leftHandOutlineColor;
                    //                }
                    //            }
                    //            else
                    //            {
                    //                if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                    //                {
                    //                    // 如果上下左右任一点颜色是255, 则该点为外部点
                    //                    if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
                    //                    {
                    //                        // 进行范围判定
                    //                        if (pixelX <= leftHandPixelXMax && pixelX >= leftHandPixelXMin && pixelY <= leftHandPixelYMax && pixelY >= leftHandPixelYMin)
                    //                        {
                    //                            this.bodyIndexPixels[i] = leftHandOutlineColor;

                    //                            // 补充手掌点其他信息
                    //                            hp.Type = HandPointType.LeftStraighten;
                    //                            hp.IsOutlinePoint = true;
                    //                            leftHandStraightenPts.Add(hp);
                    //                            tempLeftHandStraightenPts.Add(hp);
                    //                        }
                    //                    }
                    //                }
                    //            }                           
                    //            break;
                    //        case 2:         // 左手弯曲
                    //                        // 进行外部点的判断
                    //            if (radShowAll.IsChecked == true)
                    //            {
                    //                // 进行范围判定
                    //                if (pixelX <= leftHandPixelXMax && pixelX >= leftHandPixelXMin && pixelY <= leftHandPixelYMax && pixelY >= leftHandPixelYMin)
                    //                {
                    //                    this.bodyIndexPixels[i] = leftBendHandOutlineColor;
                    //                }
                    //            }
                    //            else
                    //            {
                    //                if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                    //                {
                    //                    // 如果上下左右任一点颜色是255, 则该点为外部点
                    //                    if (handPixels[i - 512] == 1 || handPixels[i - 1] == 1 || handPixels[i + 1] == 1 || handPixels[i + 512] == 1)
                    //                    {
                    //                        // 进行范围判定
                    //                        if (pixelX <= leftHandPixelXMax && pixelX >= leftHandPixelXMin && pixelY <= leftHandPixelYMax && pixelY >= leftHandPixelYMin)
                    //                        {
                    //                            this.bodyIndexPixels[i] = leftBendHandOutlineColor;

                    //                            // 补充手掌点其他信息
                    //                            hp.Type = HandPointType.LeftBend;
                    //                            hp.IsOutlinePoint = true;
                    //                            leftHandBendPts.Add(hp);
                    //                            tempLeftHandBendPts.Add(hp);
                    //                        }
                    //                    }
                    //                }
                    //            }
                    //            break;
                    //        case 3:         // 右手
                    //                        // 进行外部点的判断
                    //            if (chcShowBothHands.IsChecked == false)
                    //            {
                    //                continue;
                    //            }
                    //            if (radShowAll.IsChecked == true)
                    //            {
                    //                // 进行范围判定
                    //                if (pixelX <= rightHandPixelXMax && pixelX >= rightHandPixelXMin && pixelY <= rightHandPixelYMax && pixelY >= rightHandPixelYMin)
                    //                {
                    //                    this.bodyIndexPixels[i] = rightHandOutlineColor;
                    //                }
                    //            }
                    //            else
                    //            {
                    //                if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                    //                {
                    //                    // 如果上下左右任一点颜色是255, 则该点为外部点
                    //                    if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
                    //                    {
                    //                        // 进行范围判定
                    //                        if (pixelX <= rightHandPixelXMax && pixelX >= rightHandPixelXMin && pixelY <= rightHandPixelYMax && pixelY >= rightHandPixelYMin)
                    //                        {
                    //                            this.bodyIndexPixels[i] = rightHandOutlineColor;

                    //                            // 补充手掌点其他信息
                    //                            hp.Type = HandPointType.RightStraighten;
                    //                            hp.IsOutlinePoint = true;
                    //                            rightHandStraightenPts.Add(hp);
                    //                            tempRightHandStraightenPts.Add(hp);
                    //                        }
                    //                    }
                    //                }
                    //            }
                    //            break;
                    //        case 4:         // 右手弯曲
                    //                        // 进行外部点的判断 
                    //            if (chcShowBothHands.IsChecked == false)
                    //            {
                    //                continue;
                    //            }
                    //            if (radShowAll.IsChecked == true)
                    //            {
                    //                // 进行范围判定
                    //                if (pixelX <= rightHandPixelXMax && pixelX >= rightHandPixelXMin && pixelY <= rightHandPixelYMax && pixelY >= rightHandPixelYMin)
                    //                {
                    //                    this.bodyIndexPixels[i] = rightBendHandOutlineColor;
                    //                }
                    //            }
                    //            else
                    //            {
                    //                if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                    //                {
                    //                    // 如果上下左右任一点颜色是255, 则该点为外部点
                    //                    if (handPixels[i - 512] == 3 || handPixels[i - 1] == 3 || handPixels[i + 1] == 3 || handPixels[i + 512] == 3)
                    //                    {
                    //                        // 进行范围判定
                    //                        if (pixelX <= rightHandPixelXMax && pixelX >= rightHandPixelXMin && pixelY <= rightHandPixelYMax && pixelY >= rightHandPixelYMin)
                    //                        {
                    //                            this.bodyIndexPixels[i] = rightBendHandOutlineColor;

                    //                            // 补充手掌点其他信息
                    //                            hp.Type = HandPointType.RightBend;
                    //                            hp.IsOutlinePoint = true;
                    //                            rightHandBendPts.Add(hp);
                    //                            tempRightHandBendPts.Add(hp);
                    //                        }
                    //                    }
                    //                }
                    //            }
                    //            break;
                    //        default:
                    //            this.bodyIndexPixels[i] = 0x00000000;
                    //            break;
                    //    }

                    // 测试, 显示手掌中心点
                    if (i == leftHandCenterPixel)
                    {
                        this.bodyIndexPixels[i] = 0x00FFFFFF;
                        this.bodyIndexPixels[i - 512] = 0x00FFFFFF;
                        this.bodyIndexPixels[i - 1] = 0x00FFFFFF;
                        this.bodyIndexPixels[i + 512] = 0x00FFFFFF;
                        this.bodyIndexPixels[i + 1] = 0x00FFFFFF;
                        this.bodyIndexPixels[i - 513] = 0x00FFFFFF;
                        this.bodyIndexPixels[i - 2] = 0x00FFFFFF;
                        this.bodyIndexPixels[i + 513] = 0x00FFFFFF;
                        this.bodyIndexPixels[i + 2] = 0x00FFFFFF;
                    }
                    else if (chcShowBothHands.IsChecked == true && i == rightHandCenterPixel)
                    {
                        this.bodyIndexPixels[i] = 0x00FFFFFF;
                        this.bodyIndexPixels[i - 512] = 0x00FFFFFF;
                        this.bodyIndexPixels[i - 1] = 0x00FFFFFF;
                        this.bodyIndexPixels[i + 512] = 0x00FFFFFF;
                        this.bodyIndexPixels[i + 1] = 0x00FFFFFF;
                        this.bodyIndexPixels[i - 513] = 0x00FFFFFF;
                        this.bodyIndexPixels[i - 2] = 0x00FFFFFF;
                        this.bodyIndexPixels[i + 513] = 0x00FFFFFF;
                        this.bodyIndexPixels[i + 2] = 0x00FFFFFF;
                    }
                }
                else
                {
                    // this pixel is not part of a player
                    // display black
                    //this.bodyIndexPixels[i] = 0x00000000;
                }
            }

            // 计算轮廓点
            HandPoint startPt = new HandPoint();
            int pixelX = 0, pixelY = 0;
            Utility.PixelIndexToScreenPoint(startIndex, ref pixelX, ref pixelY);
            startPt.index = startIndex;
            startPt.X = pixelX;
            startPt.Y = pixelY;
            startPt.D = this.depthPixels[startIndex];
            startPt.Type = HandPointType.LeftStraighten;
            startPt.IsOutlinePoint = true;

            // 初始化保存数组
            leftHandStraightenPts = new List<HandPoint>();
            leftHandStraightenPts.Add(startPt);

            // 获取上方的点作为判断基准点
            //int curContourPtIndex = 0;
            //int curBasePtIndex = startIndex;
            //int basePosition = 0;   // 0: i - 512, 1: i - 511, 2: i + 1, 3: i + 513, 4: i + 512, 5: i + 511, 6: i - 1, 7: i - 513
            //HandPoint curPt = null;
            //do
            //{
            //    // 判断起始点类型
            //    switch (basePosition)
            //    {
            //        case 0:
            //            // 顺时针遍历判断基准点周围四周的八个点
            //            if (handPixels[curBasePtIndex - 512] == 1 && frameData[curBasePtIndex - 512] < BodyColor.Length && this.depthPixels[curBasePtIndex - 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 512;
            //                curBasePtIndex = curBasePtIndex - 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 1;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 511] == 1 && frameData[curBasePtIndex - 511] < BodyColor.Length && this.depthPixels[curBasePtIndex - 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 511;
            //                curBasePtIndex = curBasePtIndex - 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 2;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 1] == 1 && frameData[curBasePtIndex + 1] < BodyColor.Length && this.depthPixels[curBasePtIndex + 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 1;
            //                curBasePtIndex = curBasePtIndex + 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 3;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 513] == 1 && frameData[curBasePtIndex + 513] < BodyColor.Length && this.depthPixels[curBasePtIndex + 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 513;
            //                curBasePtIndex = curBasePtIndex + 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 4;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 512] == 1 && frameData[curBasePtIndex + 512] < BodyColor.Length && this.depthPixels[curBasePtIndex + 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 512;
            //                curBasePtIndex = curBasePtIndex + 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 5;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 511] == 1 && frameData[curBasePtIndex + 511] < BodyColor.Length && this.depthPixels[curBasePtIndex + 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 511;
            //                curBasePtIndex = curBasePtIndex + 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 6;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 1] == 1 && frameData[curBasePtIndex - 1] < BodyColor.Length && this.depthPixels[curBasePtIndex - 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 1;
            //                curBasePtIndex = curBasePtIndex - 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 7;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 513] == 1 && frameData[curBasePtIndex - 513] < BodyColor.Length && this.depthPixels[curBasePtIndex - 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 513;
            //                curBasePtIndex = curBasePtIndex - 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 0;
            //                continue;
            //            }
            //            break;
            //        case 1:
            //            // 顺时针遍历判断基准点周围四周的八个点
            //            if (handPixels[curBasePtIndex - 511] == 1 && frameData[curBasePtIndex - 511] < BodyColor.Length && this.depthPixels[curBasePtIndex - 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 511;
            //                curBasePtIndex = curBasePtIndex - 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 2;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 1] == 1 && frameData[curBasePtIndex + 1] < BodyColor.Length && this.depthPixels[curBasePtIndex + 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 1;
            //                curBasePtIndex = curBasePtIndex + 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 3;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 513] == 1 && frameData[curBasePtIndex + 513] < BodyColor.Length && this.depthPixels[curBasePtIndex + 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 513;
            //                curBasePtIndex = curBasePtIndex + 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 4;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 512] == 1 && frameData[curBasePtIndex + 512] < BodyColor.Length && this.depthPixels[curBasePtIndex + 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 512;
            //                curBasePtIndex = curBasePtIndex + 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 5;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 511] == 1 && frameData[curBasePtIndex + 511] < BodyColor.Length && this.depthPixels[curBasePtIndex + 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 511;
            //                curBasePtIndex = curBasePtIndex + 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 6;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 1] == 1 && frameData[curBasePtIndex - 1] < BodyColor.Length && this.depthPixels[curBasePtIndex - 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 1;
            //                curBasePtIndex = curBasePtIndex - 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 7;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 513] == 1 && frameData[curBasePtIndex - 513] < BodyColor.Length && this.depthPixels[curBasePtIndex - 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 513;
            //                curBasePtIndex = curBasePtIndex - 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 0;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 512] == 1 && frameData[curBasePtIndex - 512] < BodyColor.Length && this.depthPixels[curBasePtIndex - 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 512;
            //                curBasePtIndex = curBasePtIndex - 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 1;
            //                continue;
            //            }
            //            break;
            //        case 2:
            //            // 顺时针遍历判断基准点周围四周的八个点
            //            if (handPixels[curBasePtIndex + 1] == 1 && frameData[curBasePtIndex + 1] < BodyColor.Length && this.depthPixels[curBasePtIndex + 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 1;
            //                curBasePtIndex = curBasePtIndex + 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 3;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 513] == 1 && frameData[curBasePtIndex + 513] < BodyColor.Length && this.depthPixels[curBasePtIndex + 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 513;
            //                curBasePtIndex = curBasePtIndex + 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 4;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 512] == 1 && frameData[curBasePtIndex + 512] < BodyColor.Length && this.depthPixels[curBasePtIndex + 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 512;
            //                curBasePtIndex = curBasePtIndex + 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 5;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 511] == 1 && frameData[curBasePtIndex + 511] < BodyColor.Length && this.depthPixels[curBasePtIndex + 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 511;
            //                curBasePtIndex = curBasePtIndex + 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 6;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 1] == 1 && frameData[curBasePtIndex - 1] < BodyColor.Length && this.depthPixels[curBasePtIndex - 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 1;
            //                curBasePtIndex = curBasePtIndex - 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 7;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 513] == 1 && frameData[curBasePtIndex - 513] < BodyColor.Length && this.depthPixels[curBasePtIndex - 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 513;
            //                curBasePtIndex = curBasePtIndex - 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 0;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 512] == 1 && frameData[curBasePtIndex - 512] < BodyColor.Length && this.depthPixels[curBasePtIndex - 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 512;
            //                curBasePtIndex = curBasePtIndex - 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 1;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 511] == 1 && frameData[curBasePtIndex - 511] < BodyColor.Length && this.depthPixels[curBasePtIndex - 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 511;
            //                curBasePtIndex = curBasePtIndex - 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 2;
            //                continue;
            //            }
            //            break;
            //        case 3:
            //            // 顺时针遍历判断基准点周围四周的八个点
            //            if (handPixels[curBasePtIndex + 513] == 1 && frameData[curBasePtIndex + 513] < BodyColor.Length && this.depthPixels[curBasePtIndex + 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 513;
            //                curBasePtIndex = curBasePtIndex + 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 4;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 512] == 1 && frameData[curBasePtIndex + 512] < BodyColor.Length && this.depthPixels[curBasePtIndex + 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 512;
            //                curBasePtIndex = curBasePtIndex + 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 5;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 511] == 1 && frameData[curBasePtIndex + 511] < BodyColor.Length && this.depthPixels[curBasePtIndex + 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 511;
            //                curBasePtIndex = curBasePtIndex + 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 6;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 1] == 1 && frameData[curBasePtIndex - 1] < BodyColor.Length && this.depthPixels[curBasePtIndex - 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 1;
            //                curBasePtIndex = curBasePtIndex - 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 7;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 513] == 1 && frameData[curBasePtIndex - 513] < BodyColor.Length && this.depthPixels[curBasePtIndex - 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 513;
            //                curBasePtIndex = curBasePtIndex - 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 0;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 512] == 1 && frameData[curBasePtIndex - 512] < BodyColor.Length && this.depthPixels[curBasePtIndex - 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 512;
            //                curBasePtIndex = curBasePtIndex - 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 1;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 511] == 1 && frameData[curBasePtIndex - 511] < BodyColor.Length && this.depthPixels[curBasePtIndex - 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 511;
            //                curBasePtIndex = curBasePtIndex - 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 2;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 1] == 1 && frameData[curBasePtIndex + 1] < BodyColor.Length && this.depthPixels[curBasePtIndex + 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 1;
            //                curBasePtIndex = curBasePtIndex + 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 3;
            //                continue;
            //            }
            //            break;
            //        case 4:
            //            // 顺时针遍历判断基准点周围四周的八个点
            //            if (handPixels[curBasePtIndex + 512] == 1 && frameData[curBasePtIndex + 512] < BodyColor.Length && this.depthPixels[curBasePtIndex + 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 512;
            //                curBasePtIndex = curBasePtIndex + 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 5;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 511] == 1 && frameData[curBasePtIndex + 511] < BodyColor.Length && this.depthPixels[curBasePtIndex + 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 511;
            //                curBasePtIndex = curBasePtIndex + 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 6;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 1] == 1 && frameData[curBasePtIndex - 1] < BodyColor.Length && this.depthPixels[curBasePtIndex - 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 1;
            //                curBasePtIndex = curBasePtIndex - 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 7;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 513] == 1 && frameData[curBasePtIndex - 513] < BodyColor.Length && this.depthPixels[curBasePtIndex - 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 513;
            //                curBasePtIndex = curBasePtIndex - 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 0;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 512] == 1 && frameData[curBasePtIndex - 512] < BodyColor.Length && this.depthPixels[curBasePtIndex - 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 512;
            //                curBasePtIndex = curBasePtIndex - 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 1;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 511] == 1 && frameData[curBasePtIndex - 511] < BodyColor.Length && this.depthPixels[curBasePtIndex - 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 511;
            //                curBasePtIndex = curBasePtIndex - 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 2;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 1] == 1 && frameData[curBasePtIndex + 1] < BodyColor.Length && this.depthPixels[curBasePtIndex + 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 1;
            //                curBasePtIndex = curBasePtIndex + 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 3;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 513] == 1 && frameData[curBasePtIndex + 513] < BodyColor.Length && this.depthPixels[curBasePtIndex + 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 513;
            //                curBasePtIndex = curBasePtIndex + 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 4;
            //                continue;
            //            }
            //            break;
            //        case 5:
            //            // 顺时针遍历判断基准点周围四周的八个点
            //            if (handPixels[curBasePtIndex + 511] == 1 && frameData[curBasePtIndex + 511] < BodyColor.Length && this.depthPixels[curBasePtIndex + 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 511;
            //                curBasePtIndex = curBasePtIndex + 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 6;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 1] == 1 && frameData[curBasePtIndex - 1] < BodyColor.Length && this.depthPixels[curBasePtIndex - 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 1;
            //                curBasePtIndex = curBasePtIndex - 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 7;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 513] == 1 && frameData[curBasePtIndex - 513] < BodyColor.Length && this.depthPixels[curBasePtIndex - 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 513;
            //                curBasePtIndex = curBasePtIndex - 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 0;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 512] == 1 && frameData[curBasePtIndex - 512] < BodyColor.Length && this.depthPixels[curBasePtIndex - 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 512;
            //                curBasePtIndex = curBasePtIndex - 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 1;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 511] == 1 && frameData[curBasePtIndex - 511] < BodyColor.Length && this.depthPixels[curBasePtIndex - 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 511;
            //                curBasePtIndex = curBasePtIndex - 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 2;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 1] == 1 && frameData[curBasePtIndex + 1] < BodyColor.Length && this.depthPixels[curBasePtIndex + 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 1;
            //                curBasePtIndex = curBasePtIndex + 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 3;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 513] == 1 && frameData[curBasePtIndex + 513] < BodyColor.Length && this.depthPixels[curBasePtIndex + 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 513;
            //                curBasePtIndex = curBasePtIndex + 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 4;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 512] == 1 && frameData[curBasePtIndex + 512] < BodyColor.Length && this.depthPixels[curBasePtIndex + 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 512;
            //                curBasePtIndex = curBasePtIndex + 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 5;
            //                continue;
            //            }
            //            break;
            //        case 6:
            //            // 顺时针遍历判断基准点周围四周的八个点
            //            if (handPixels[curBasePtIndex - 1] == 1 && frameData[curBasePtIndex - 1] < BodyColor.Length && this.depthPixels[curBasePtIndex - 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 1;
            //                curBasePtIndex = curBasePtIndex - 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 7;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 513] == 1 && frameData[curBasePtIndex - 513] < BodyColor.Length && this.depthPixels[curBasePtIndex - 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 513;
            //                curBasePtIndex = curBasePtIndex - 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 0;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 512] == 1 && frameData[curBasePtIndex - 512] < BodyColor.Length && this.depthPixels[curBasePtIndex - 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 512;
            //                curBasePtIndex = curBasePtIndex - 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 1;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 511] == 1 && frameData[curBasePtIndex - 511] < BodyColor.Length && this.depthPixels[curBasePtIndex - 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 511;
            //                curBasePtIndex = curBasePtIndex - 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 2;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 1] == 1 && frameData[curBasePtIndex + 1] < BodyColor.Length && this.depthPixels[curBasePtIndex + 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 1;
            //                curBasePtIndex = curBasePtIndex + 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 3;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 513] == 1 && frameData[curBasePtIndex + 513] < BodyColor.Length && this.depthPixels[curBasePtIndex + 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 513;
            //                curBasePtIndex = curBasePtIndex + 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 4;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 512] == 1 && frameData[curBasePtIndex + 512] < BodyColor.Length && this.depthPixels[curBasePtIndex + 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 512;
            //                curBasePtIndex = curBasePtIndex + 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 5;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 511] == 1 && frameData[curBasePtIndex + 511] < BodyColor.Length && this.depthPixels[curBasePtIndex + 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 511;
            //                curBasePtIndex = curBasePtIndex + 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 6;
            //                continue;
            //            }
            //            break;
            //        case 7:
            //            // 顺时针遍历判断基准点周围四周的八个点
            //            if (handPixels[curBasePtIndex - 513] == 1 && frameData[curBasePtIndex - 513] < BodyColor.Length && this.depthPixels[curBasePtIndex - 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 513;
            //                curBasePtIndex = curBasePtIndex - 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 0;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 512] == 1 && frameData[curBasePtIndex - 512] < BodyColor.Length && this.depthPixels[curBasePtIndex - 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 512;
            //                curBasePtIndex = curBasePtIndex - 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 1;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 511] == 1 && frameData[curBasePtIndex - 511] < BodyColor.Length && this.depthPixels[curBasePtIndex - 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 511;
            //                curBasePtIndex = curBasePtIndex - 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 2;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 1] == 1 && frameData[curBasePtIndex + 1] < BodyColor.Length && this.depthPixels[curBasePtIndex + 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 1;
            //                curBasePtIndex = curBasePtIndex + 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 3;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 513] == 1 && frameData[curBasePtIndex + 513] < BodyColor.Length && this.depthPixels[curBasePtIndex + 513] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 513;
            //                curBasePtIndex = curBasePtIndex + 513;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 4;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 512] == 1 && frameData[curBasePtIndex + 512] < BodyColor.Length && this.depthPixels[curBasePtIndex + 512] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 512;
            //                curBasePtIndex = curBasePtIndex + 512;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 5;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex + 511] == 1 && frameData[curBasePtIndex + 511] < BodyColor.Length && this.depthPixels[curBasePtIndex + 511] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex + 511;
            //                curBasePtIndex = curBasePtIndex + 511;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 6;
            //                continue;
            //            }
            //            else if (handPixels[curBasePtIndex - 1] == 1 && frameData[curBasePtIndex - 1] < BodyColor.Length && this.depthPixels[curBasePtIndex - 1] != 0)
            //            {
            //                curContourPtIndex = curBasePtIndex - 1;
            //                curBasePtIndex = curBasePtIndex - 1;
            //                this.bodyIndexPixels[curContourPtIndex] = leftHandOutlineColor;

            //                curPt = new HandPoint();
            //                Utility.PixelIndexToScreenPoint(curContourPtIndex, ref pixelX, ref pixelY);
            //                curPt.index = curContourPtIndex;
            //                curPt.X = pixelX;
            //                curPt.Y = pixelY;
            //                curPt.D = this.depthPixels[curContourPtIndex];
            //                curPt.Type = HandPointType.LeftStraighten;
            //                curPt.IsOutlinePoint = true;
            //                leftHandStraightenPts.Add(curPt);
            //                basePosition = 7;
            //                continue;
            //            }
            //            break;
            //    }


            //}
            //while (curContourPtIndex != startIndex && curContourPtIndex != startIndex + 512 && curContourPtIndex != startIndex + 511 && curContourPtIndex != startIndex + 513 && curContourPtIndex != startIndex - 1);
            int curContourIndex = startIndex;
            int lastContourIndex = startIndex - 512;
            int startTracingIndex = startIndex - 512;
            do
            {

            }
            while (curContourIndex != startIndex);


        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // 计算左右手限制范围
            int leftHandPixelXMin = leftHandCenterPixelX - handPixelLimitedWidth;
            int leftHandPixelXMax = leftHandCenterPixelX + handPixelLimitedWidth;
            int leftHandPixelYMin = leftHandCenterPixelY - handPixelLimitedHeight;
            int leftHandPixelYMax = leftHandCenterPixelY + handPixelLimitedHeight;

            int rightHandPixelXMin = rightHandCenterPixelX - handPixelLimitedWidth;
            int rightHandPixelXMax = rightHandCenterPixelX + handPixelLimitedWidth;
            int rightHandPixelYMin = rightHandCenterPixelY - handPixelLimitedHeight;
            int rightHandPixelYMax = rightHandCenterPixelY + handPixelLimitedHeight;

            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;
            int count = 0;

            this.handPixels = new int[(int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel)];

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
                if (depth > this.maxHandDist || depth == 0)
                {
                    this.depthPixels[i] = 0;
                    this.handPixels[i] = 0;
                }
                else
                {
                    int pixelX = 0, pixelY = 0;
                    Utility.PixelIndexToScreenPoint(i, ref pixelX, ref pixelY);
                    if (chcShowBothHands.IsChecked == true)
                    {
                        // 进行左右手像素点区分
                        if (maxDistHand == 1)   // 左手
                        {
                            if (depth > maxHandDist - bendFigerhreshold)                                        // 判断是否为左手像素点
                            {
                                this.handPixels[i] = 1;
                            }
                            else if (depth <= maxHandDist - bendFigerhreshold && depth > rightWristDist)        // 判断是否为左手弯曲部分像素点
                            {
                                this.handPixels[i] = 2;
                            }
                            else if (depth <= rightWristDist && depth > rightWristDist - bendFigerhreshold)     // 判断是否为右手像素点
                            {
                                this.handPixels[i] = 3;
                            }
                            else if (depth <= rightWristDist - bendFigerhreshold)                               // 判断是否为右手弯曲部分像素点
                            {
                                this.handPixels[i] = 4;
                            }
                        }
                        else if (maxDistHand == 2)  // 右手
                        {
                            if (depth > maxHandDist - bendFigerhreshold)                                        // 判断是否为右手像素点
                            {
                                this.handPixels[i] = 3;
                            }
                            else if (depth <= maxHandDist - bendFigerhreshold && depth > leftWristDist)         // 判断是否为右手弯曲部分
                            {
                                this.handPixels[i] = 4;
                            }
                            else if (depth <= leftWristDist && depth > leftWristDist - bendFigerhreshold)       // 判断是否为左手像素点
                            {
                                this.handPixels[i] = 1;
                            }
                            else if (depth <= leftWristDist - bendFigerhreshold)                                // 判断是否为左手弯曲部分像素点
                            {
                                this.handPixels[i] = 2;
                            }
                        }
                        else
                        {
                            this.handPixels[i] = 0;
                        }
                    }
                    else
                    {
                        // 只显示左手
                        if (depth <= leftWristDist && depth > leftWristDist - bendFigerhreshold)
                        {
                            if (pixelX <= leftHandPixelXMax && pixelX >= leftHandPixelXMin && pixelY <= leftHandPixelYMax && pixelY >= leftHandPixelYMin)
                            {
                                this.handPixels[i] = 1;
                            }
                                
                        }
                        else if (depth <= leftWristDist - bendFigerhreshold)
                        {
                            if (pixelX <= leftHandPixelXMax && pixelX >= leftHandPixelXMin && pixelY <= leftHandPixelYMax && pixelY >= leftHandPixelYMin)
                            {
                                this.handPixels[i] = 2;
                            }
                        }
                        else
                        {
                            this.handPixels[i] = 0;
                        }
                    }
                    count++;
                }







                //                 if ((this.depthPixels[i] >= this.leftMiniDist && this.depthPixels[i] <= this.leftMaxDist) || (this.depthPixels[i] >= this.rightMiniDist && this.depthPixels[i] <= this.rightMaxDist))
                //                 {
                //                     // To convert to a byte, we're mapping the depth value to the byte range.
                //                     // Values outside the reliable depth range are mapped to 0 (black).
                //                     this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
                //                 }
                //                 else
                //                 {
                //                     this.depthPixels[i] = 0;
                //                 }
            }
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderBodyIndexPixels()
        {
            this.bodyIndexBitmap.WritePixels(
                new Int32Rect(0, 0, this.bodyIndexBitmap.PixelWidth, this.bodyIndexBitmap.PixelHeight),
                this.bodyIndexPixels,
                this.bodyIndexBitmap.PixelWidth * (int)BytesPerPixel,
                0);

            image.Source = this.bodyIndexBitmap;
        }


        /// <summary>
        /// 计算手部像素点
        /// </summary>
        /// <param name="distance"></param>
        private void GetHandPiexlCount(float distance)
        {
            
        }

        private void btnSaveImage_Click(object sender, RoutedEventArgs e)
        {
            if (this.bodyIndexBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(this.bodyIndexBitmap));

                string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

                string path = System.IO.Path.Combine(myPhotos, "KinectScreenshot-BodyIndex-" + time + ".png");

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
                    using (FileStream fs = new FileStream(path, FileMode.Create))
                    {
                        encoder.Save(fs);
                    }

                }
                catch (IOException)
                {

                }
            }
        }

        private void radOutline_Checked(object sender, RoutedEventArgs e)
        {

        }
    }
}
