#coding=utf-8
import sys, rospy, cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import PySpin
import time
import matplotlib.pyplot as plt
NUM_IMAGES = 1  # number of images to grab


def acquire_images(cam, nodemap, nodemap_tldevice):

    print "*** IMAGE ACQUISITION ***\n"
    try:
        result = True
        # In order to access the node entries, they have to be casted to a pointer type (CEnumerationPtr here)
        node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionMode"))
        if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
            print "Unable to set acquisition mode to continuous (enum retrieval). Aborting..."
            return False
        # Retrieve entry node from enumeration node
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName("Continuous")
        if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(node_acquisition_mode_continuous):
            print "Unable to set acquisition mode to continuous (entry retrieval). Aborting..."
            return False
        # Retrieve integer value from entry node
        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

        # Set integer value from entry node as new value of enumeration node
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

        print "Acquisition mode set to continuous..."

        cam.BeginAcquisition()

        print "Acquiring images..."
        #  this.
        device_serial_number = ""
        node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode("DeviceSerialNumber"))
        if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
            device_serial_number = node_device_serial_number.GetValue()
            print "Device serial number retrieved as %s..." % device_serial_number

        # Retrieve, convert, and save images
        for i in range(NUM_IMAGES):
            try:
                #  buffer from filling up.
                image_result = cam.GetNextImage()
                if image_result.IsIncomplete():
                    print "Image incomplete with image status %d ..." % image_result.GetImageStatus()

                else:
                    width = image_result.GetWidth()
                    height = image_result.GetHeight()

                    print "Grabbed Image %d, width = %d, height = %d" % (i, width, height)
                    img=image_result.GetNDArray()
                    #cv2.imwrite('raw.jpg',img)

                    #  Convert image to mono 8
                    #  optional parameter.
                    image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)
                    print("----------------------------------")
                    print(image_converted.GetWidth())
                    print("----------------------------------")
                    # Create a unique filename
                    if device_serial_number:
                        filename = "Acquisition-%s-%d.jpg" % (device_serial_number, i)
                    else:  # if serial number is empty
                        t=time.time()
                        filename = "Acquisition-%d-%d.jpg" % (t,i)
                    #  Save image
                    #image_converted.Save(filename)
                    print "Image saved at %s" % filename
                    #  Release image
                    image_result.Release()
                    print ""

            except PySpin.SpinnakerException as ex:
                print "Error: %s" % ex
                return False
        #  End acquisition
        cam.EndAcquisition()

    except PySpin.SpinnakerException as ex:
        print "Error: %s" % ex
        return False

    return result,img


def print_device_info(nodemap):

    print "*** DEVICE INFORMATION ***\n"
    try:
        result = True
        node_device_information = PySpin.CCategoryPtr(nodemap.GetNode("DeviceInformation"))

        if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
            features = node_device_information.GetFeatures()
            for feature in features:
                node_feature = PySpin.CValuePtr(feature)
                print "%s: %s" % (node_feature.GetName(),
                                  node_feature.ToString() if PySpin.IsReadable(node_feature) else "Node not readable")

        else:
            print "Device control information not available."

    except PySpin.SpinnakerException as ex:
        print "Error: %s" % ex
        return False

    return result

def run_single_camera(cam):

    try:
        result = True

        # Retrieve TL device nodemap and print device information
        nodemap_tldevice = cam.GetTLDeviceNodeMap()

        result &= print_device_info(nodemap_tldevice)

        # Initialize camera
        cam.Init()

        # Retrieve GenICam nodemap
        nodemap = cam.GetNodeMap()

        # Acquire images
        result1,img=acquire_images(cam, nodemap, nodemap_tldevice)
        result &= result1

        # Deinitialize camera
        cam.DeInit()

    except PySpin.SpinnakerException as ex:
        print "Error: %s" % ex
        result = False

    return result,img
    print("run signle camera jieshu")


#



def main(args):

    # Retrieve singleton reference to system object
    system = PySpin.System.GetInstance()

    # Retrieve list of cameras from the system
    cam_list = system.GetCameras()

    num_cameras = cam_list.GetSize()

    print "Number of cameras detected: %d" % num_cameras

    # Finish if there are no cameras
    if num_cameras == 0:

        # Clear camera list before releasing system
        cam_list.Clear()

        # Release system
        system.ReleaseInstance()

        print "Not enough cameras!"
        #raw_input("Done! Press Enter to exit...")
        return False

    # Run example on each camera
    for i in range(num_cameras):


        cam = cam_list.GetByIndex(i)

        print "Running example for camera %d..." % i



        pub = rospy.Publisher('/pg_camera/image', Image, queue_size=1)
        rospy.init_node('pg_driver', anonymous=True)
        rate = rospy.Rate(10)
        bridge = CvBridge()

        while not rospy.is_shutdown():
            #cv2.imshow("image", img)
            #cv2.waitKey(3)
            result,img = run_single_camera(cam)
            cv2.resize(img, (320, 240), interpolation=cv2.INTER_CUBIC)

            pub.publish(bridge.cv2_to_imgmsg(img, "8UC1"))
            rate.sleep()


        print "Camera %d example complete..." % i

    # Release reference to camera
    # NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
    # cleaned up when going out of scope.
    # The usage of del is preferred to assigning the variable to None.
    del cam

    # Clear camera list before releasing system
    cam_list.Clear()

    # Release instance
    system.ReleaseInstance()

    #raw_input("Done! Press Enter to exit...")
    return result

if __name__ == "__main__":
    main(sys.argv)
