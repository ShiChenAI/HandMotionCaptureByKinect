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
        private static readonly uint leftHandOutlineColor = 0xFF33FF00;

        /// <summary>
        /// 左手弯曲手指轮廓颜色
        /// </summary>
        private static readonly uint leftBendHandOutlineColor = 0xFF808000;

        /// <summary>
        /// 右手外围轮廓颜色
        /// </summary>
        private static readonly uint rightHandOutlineColor = 0xFF40FF00;

        /// <summary>
        /// 右手弯曲手指轮廓颜色
        /// </summary>
        private static readonly uint rightBendHandOutlineColor = 0xFFFF4000;

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
        private int handPixelLimitedWidth = 85;

        /// <summary>
        /// 手的像素点限制区域高度
        /// </summary>
        private int handPixelLimitedHeight = 106;

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
        private double bendFigerhreshold = 5;

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
                    // Utility.ScreenPointPixelIndex((int)depthSpacePoint.X, (int)depthSpacePoint.Y, ref leftHandCenterPixel);
                    leftHandCenterPixelX = (int)depthSpacePoint.X;
                    leftHandCenterPixelY = (int)depthSpacePoint.Y;

                    position = joints[JointType.HandRight].Position;
                    depthSpacePoint = this.mapper.MapCameraPointToDepthSpace(position);
                    // Utility.ScreenPointPixelIndex((int)depthSpacePoint.X, (int)depthSpacePoint.Y, ref rightHandCenterPixel); 
                    rightHandCenterPixelX = (int)depthSpacePoint.X;
                    rightHandCenterPixelY = (int)depthSpacePoint.Y;
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

            byte* frameData = (byte*)bodyIndexFrameData;

            for (int i = 0; i < this.handPixels.Count(); i++)
            {
               
                if (frameData[i] < BodyColor.Length && this.depthPixels[i] != 0)
                {
                    this.bodyIndexPixels[i] = 0x00000000;
                    int pixelX = 0, pixelY = 0;
                    Utility.PixelIndexToScreenPoint(i, ref pixelX, ref pixelY);

                    switch (handPixels[i])
                    {
                        case 1:         // 左手
                            // 进行外部点的判断                  
                            if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                            {
                                // 如果上下左右任一点颜色是255, 则该点为外部点
                                if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
                                {
                                    // 进行范围判定
                                    if (pixelX <= leftHandPixelXMax && pixelX >= leftHandPixelXMin && pixelY <= leftHandPixelYMax && pixelY >= leftHandPixelYMin)
                                    {
                                        this.bodyIndexPixels[i] = leftHandOutlineColor;
                                    }
                                }
                            }
                            break;
                        case 2:         // 左手弯曲
                            // 进行外部点的判断                  
                            if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                            {
                                // 如果上下左右任一点颜色是255, 则该点为外部点
                                if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
                                {
                                    // 进行范围判定
                                    if (pixelX <= leftHandPixelXMax && pixelX >= leftHandPixelXMin && pixelY <= leftHandPixelYMax && pixelY >= leftHandPixelYMin)
                                    {
                                        this.bodyIndexPixels[i] = leftBendHandOutlineColor;
                                    }
                                }
                            }
                            break;
                        case 3:         // 右手
                            // 进行外部点的判断                  
                            if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                            {
                                // 如果上下左右任一点颜色是255, 则该点为外部点
                                if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
                                {
                                    // 进行范围判定
                                    if (pixelX <= rightHandPixelXMax && pixelX >= rightHandPixelXMin && pixelY <= rightHandPixelYMax && pixelY >= rightHandPixelYMin)
                                    {
                                        this.bodyIndexPixels[i] = rightHandOutlineColor;
                                    }
                                }
                            }
                            break;
                        case 4:         // 右手弯曲
                            // 进行外部点的判断                  
                            if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                            {
                                // 如果上下左右任一点颜色是255, 则该点为外部点
                                if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
                                {
                                    // 进行范围判定
                                    if (pixelX <= rightHandPixelXMax && pixelX >= rightHandPixelXMin && pixelY <= rightHandPixelYMax && pixelY >= rightHandPixelYMin)
                                    {
                                        this.bodyIndexPixels[i] = rightBendHandOutlineColor;
                                    }
                                }
                            }
                            break;
                        default:
                            this.bodyIndexPixels[i] = 0x00000000;
                            break;
                    }

                    if (i == leftHandCenterPixel || i == rightHandCenterPixel)
                    {
                        this.bodyIndexPixels[i] = 0xFF000000;
                    }

                    //if (handPixels[i] == 0)
                    //{
                    //    this.bodyIndexPixels[i] = 0x00000000;
                    //}
                    //else if (handPixels[i] == 1)
                    //{
                    //    // 进行外部点的判断                  
                    //    if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                    //    {
                    //        // 如果上下左右任一点颜色是255, 则该点为外部点
                    //        if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
                    //        {
                    //            this.bodyIndexPixels[i] = handOutlineColor;
                    //        }
                    //    }
                    //}
                    //else if (handPixels[i] == 2)
                    //{
                    //    // 进行外部点的判断                  
                    //    if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
                    //    {
                    //        // 如果上下左右任一点颜色是255, 则该点为外部点
                    //        if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
                    //        {
                    //            this.bodyIndexPixels[i] = BendHandOutlineColor;
                    //        }
                    //    }
                    //}


                }
                else
                {
                    // this pixel is not part of a player
                    // display black
                    this.bodyIndexPixels[i] = 0x00000000;
                }
            }

            // 弯曲手指轮廓判定

            // 轮廓像素点排序












            //// 遍历左手像素点编号
            //foreach (int index in leftHandPixelInxexs)
            //{
            //    if (frameData[index] < BodyColor.Length && this.depthPixels[index] != 0)
            //    {
            //        // 进行外部点的判断                  
            //        if (index > 512 && index + 512 < (int)bodyIndexFrameDataSize)
            //        {
            //            // 如果上下左右任一点颜色是255, 则该点为外部点
            //            if (frameData[index - 512] == 255 || frameData[index - 1] == 255 || frameData[index + 1] == 255 || frameData[index + 512] == 255)
            //            {
            //                this.bodyIndexPixels[index] = handOutlineColor;
            //            }
            //        }
            //    }
            //    else
            //    {
            //        // this pixel is not part of a player
            //        // display black
            //        this.bodyIndexPixels[index] = 0x00000000;
            //    }
            //}



            //// 遍历右手像素点编号
            //foreach (int index in rightHandPixelInxexs)
            //{
            //    if (frameData[index] < BodyColor.Length && this.depthPixels[index] != 0)
            //    {
            //        // 进行外部点的判断                  
            //        if (index > 512 && index + 512 < (int)bodyIndexFrameDataSize)
            //        {
            //            // 如果上下左右任一点颜色是255, 则该点为外部点
            //            if (frameData[index - 512] == 255 || frameData[index - 1] == 255 || frameData[index + 1] == 255 || frameData[index + 512] == 255)
            //            {
            //                this.bodyIndexPixels[index] = BendHandOutlineColor;
            //            }
            //        }
            //    }
            //    else
            //    {
            //        // this pixel is not part of a player
            //        // display black
            //        this.bodyIndexPixels[index] = 0x00000000;
            //    }
            //}















            //// convert body index to a visual representation
            //for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
            //{
            //    // the BodyColor array has been sized to match
            //    // BodyFrameSource.BodyCount
            //    if (frameData[i] < BodyColor.Length && this.depthPixels[i] != 0)
            //    {
            //        // this pixel is part of a player,
            //        // display the appropriate color
            //        this.bodyIndexPixels[i] = BodyColor[frameData[i]];
            //        this.bodyIndexPixels[i] = 0;


            //        // 进行外部点的判断                  
            //        if (i > 512 && i + 512 < (int)bodyIndexFrameDataSize)
            //        {
            //            // 如果上下左右任一点颜色是255, 则该点为外部点
            //            if (frameData[i - 512] == 255 || frameData[i - 1] == 255 || frameData[i + 1] == 255 || frameData[i + 512] == 255)
            //            {
            //                this.bodyIndexPixels[i] = handOutlineColor;
            //            }
            //        }




            //        //// 根据阈值计算弯曲手指
            //        //if (this.depthPixels[i] < this.maxHandDist - 5)
            //        //{
            //        //    this.bodyIndexPixels[i] = BendHandOutlineColor;
            //        //}











            //    }
            //    else
            //    {
            //        // this pixel is not part of a player
            //        // display black
            //        this.bodyIndexPixels[i] = 0x00000000;
            //    }
            //}
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
            int ccc = 0;
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
    }
}
