/*
Show a marker using the projector, capture a set of 2D images to find the marker coordinates (2D and 3D).

This example shows how a marker can be projected onto a surface using the built-in projector. A 2D capture with
zero brightness is then used to capture an image with the marker. Finally position of the marker is detected,
allowing us to find the 3D coordinates relative to the camera.
*/
#include <Zivid/Application.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Exception.h>
#include <Zivid/Experimental/Projection.h>

#include <opencv2/opencv.hpp>

#include <cstring>
#include <iostream>

namespace
{
    cv::Mat
    createBackgroundImage(const Zivid::Resolution &resolution, int cvMatArrayType, const cv::Scalar &backgroundColor)
    {
        return cv::Mat{
            static_cast<int>(resolution.height()), static_cast<int>(resolution.width()), cvMatArrayType, backgroundColor
        };
    }

    cv::Mat createMarker(
        const Zivid::Resolution &resolution,
        int cvMatArrayType,
        const cv::Scalar &markerColor,
        const cv::Scalar &backgroundColor)
    {
        auto marker = createBackgroundImage(resolution, cvMatArrayType, backgroundColor);
        const int x = resolution.width() / 2;
        const int y = resolution.height() / 2;
        cv::circle(marker, cv::Point(x, y), 1, markerColor, -1);
        cv::line(marker, cv::Point{ x, y + 5 }, cv::Point{ x, marker.cols }, markerColor, 1);
        cv::line(marker, cv::Point{ x, y - 5 }, cv::Point{ x, 0 }, markerColor, 1);
        cv::line(marker, cv::Point{ x + 5, y }, cv::Point{ marker.rows, y }, markerColor, 1);
        cv::line(marker, cv::Point{ x - 5, y }, cv::Point{ 0, y }, markerColor, 1);

        return marker;
    }

    void copyToCenter(const cv::Mat &sourceImage, const cv::Mat &destinationImage)
    {
        const cv::Rect area{ (destinationImage.cols - sourceImage.cols) / 2,
                             (destinationImage.rows - sourceImage.rows) / 2,
                             sourceImage.cols,
                             sourceImage.rows };
        cv::Mat regionOfInterest{ destinationImage, area };
        sourceImage.copyTo(regionOfInterest);
    }

    cv::Mat cvMatFromFrame2D(const Zivid::Frame2D &frame2D)
    {
        const auto image = frame2D.imageRGBA();

        // The cast for image.data() is required because the cv::Mat constructor requires non-const void *.
        // It does not actually mutate the data, it only adds an OpenCV header to the matrix. We then protect
        // our own instance with const.
        const cv::Mat rgbaMat(
            image.height(),
            image.width(),
            CV_8UC4, // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            const_cast<void *>(static_cast<const void *>(image.data())));
        cv::Mat bgr;
        cv::cvtColor(rgbaMat, bgr, cv::COLOR_RGBA2BGR);

        return bgr;
    }

    cv::Mat croppedGrayFloatImage(const cv::Mat &bgr, int cropRows)
    {
        const auto cropped = bgr(cv::Range{ cropRows, bgr.rows - cropRows }, cv::Range{ 0, bgr.cols });

        cv::Mat grayImage;
        cv::cvtColor(cropped, grayImage, cv::COLOR_BGR2GRAY);

        cv::Mat outputImage;
        grayImage.convertTo(outputImage, CV_32F);
        return outputImage;
    }

    cv::Mat
    normalize(const cv::Mat &markerImage, const cv::Mat &illuminatedSceneImage, const cv::Mat &nonIlluminatedSceneImage)
    {
        // We use the difference between the light and dark background images to normalize the marker image
        cv::Mat difference = illuminatedSceneImage - nonIlluminatedSceneImage;

        // Avoid divide-by-zero by ignoring pixels with little value difference
        const auto differenceLimit = 100.0;
        const auto invalidDifference = difference < differenceLimit;
        difference.setTo(1, invalidDifference);

        cv::Mat normalizedImage = (markerImage - nonIlluminatedSceneImage) / difference;
        normalizedImage.setTo(0, invalidDifference);
        return normalizedImage;
    }

    double projectorToCameraScaleFactor(const Zivid::CameraInfo &cameraInfo)
    {
        // Note: these values are approximate and only for use in this demo
        switch(cameraInfo.model().value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge: break;

            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100: return 1.52;

            case Zivid::CameraInfo::Model::ValueType::zividTwoPlusM130: return 2.47;
        }
        throw std::invalid_argument("Invalid camera model");
    }

    cv::Point findMarker(
        const Zivid::Frame2D &projectedMarkerFrame2D,
        const Zivid::Frame2D &illuminatedSceneFrame2D,
        const Zivid::Frame2D &nonIlluminatedSceneFrame2D,
        const Zivid::Resolution &markerResolution,
        const Zivid::CameraInfo &cameraInfo)
    {
        constexpr int croppedRows = 400;

        const auto normalizedImage = normalize(
            croppedGrayFloatImage(cvMatFromFrame2D(projectedMarkerFrame2D), croppedRows),
            croppedGrayFloatImage(cvMatFromFrame2D(illuminatedSceneFrame2D), croppedRows),
            croppedGrayFloatImage(cvMatFromFrame2D(nonIlluminatedSceneFrame2D), croppedRows));

        cv::Mat blurredMarker;
        cv::GaussianBlur(createMarker(markerResolution, CV_32F, 1, 0), blurredMarker, cv::Size{ 5, 5 }, 1.0, 1.0);

        cv::Mat kernel;
        const auto scaleFactor = projectorToCameraScaleFactor(cameraInfo);
        cv::resize(blurredMarker, kernel, {}, scaleFactor, scaleFactor);

        cv::Mat convolvedImage;
        cv::filter2D(normalizedImage, convolvedImage, -1, kernel);

        cv::Point brightestLocation;
        cv::minMaxLoc(convolvedImage, nullptr, nullptr, nullptr, &brightestLocation);

        return brightestLocation + cv::Point{ 0, croppedRows };
    }

    cv::Mat annotate(const Zivid::Frame2D &frame2D, const cv::Point &location)
    {
        auto image = cvMatFromFrame2D(frame2D);
        const cv::Scalar markerColor{ 0, 0, 255, 255 };
        const int markerSize = 10;

        const auto topLeftLocation = location + cv::Point{ -markerSize, -markerSize };
        const auto bottomRightLocation = location + cv::Point{ markerSize, markerSize };
        cv::line(image, topLeftLocation, bottomRightLocation, markerColor, 2);

        const auto topRightLocation = location + cv::Point{ -markerSize, markerSize };
        const auto bottomLeftLocation = location + cv::Point{ markerSize, -markerSize };
        cv::line(image, topRightLocation, bottomLeftLocation, markerColor, 2);

        return image;
    }

    Zivid::Frame captureWithCaptureAssistant(Zivid::Camera &camera)
    {
        const auto suggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 1200 } }
        };
        auto settings = Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters);
        const auto processing = Zivid::Settings::Processing{
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Reflection::Removal::Experimental::Mode::global,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 }
        };
        settings.set(processing);
        return camera.capture(settings);
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Retrieving the projector resolution that the camera supports" << std::endl;
        const auto projectorResolution = Zivid::Experimental::Projection::projectorResolution(camera);

        std::cout << "Creating a projector image with resolution: " << projectorResolution.toString() << std::endl;
        const int cvMatArrayType = CV_8UC4;
        const cv::Scalar backgroundColor{ 0, 0, 0, 255 };
        auto projectorImageOpenCV = createBackgroundImage(projectorResolution, cvMatArrayType, backgroundColor);

        std::cout << "Drawing a green marker" << std::endl;
        const Zivid::Resolution markerResolution{ 41, 41 };
        const cv::Scalar markerColor{ 0, 255, 0, 255 };
        auto marker = createMarker(markerResolution, cvMatArrayType, markerColor, backgroundColor);

        std::cout << "Copying the marker image to the projector image" << std::endl;
        copyToCenter(marker, projectorImageOpenCV);

        std::cout << "Creating a Zivid::Image from the OpenCV image" << std::endl;
        const Zivid::Image<Zivid::ColorBGRA> projectorImage{
            projectorResolution,
            reinterpret_cast<const Zivid::ColorBGRA *>(projectorImageOpenCV.datastart),
            reinterpret_cast<const Zivid::ColorBGRA *>(projectorImageOpenCV.dataend)
        };

        const std::string projectorImageFile = "ProjectorImage.png";
        std::cout << "Saving the projector image to file: " << projectorImageFile << std::endl;
        projectorImage.save(projectorImageFile);

        std::cout << "Displaying the projector image" << std::endl;
        auto projectedImageHandle = Zivid::Experimental::Projection::showImage(camera, projectorImage);

        std::cout << "Press enter to continue...";
        std::cin.get();

        const auto settings2DZeroBrightness =
            Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
                Zivid::Settings2D::Acquisition::Brightness{ 0.0 },
                Zivid::Settings2D::Acquisition::ExposureTime{ std::chrono::microseconds{ 40000 } },
                Zivid::Settings2D::Acquisition::Aperture{ 2.83 } } } };

        const auto settings2DMaxBrightness =
            Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
                Zivid::Settings2D::Acquisition::Brightness{ 1.8 },
                Zivid::Settings2D::Acquisition::ExposureTime{ std::chrono::microseconds{ 40000 } },
                Zivid::Settings2D::Acquisition::Aperture{ 2.83 } } } };

        std::cout << "Capture a 2D frame with the marker" << std::endl;
        const auto projectedMarkerFrame2D = projectedImageHandle.capture(settings2DZeroBrightness);

        std::cout << "Capture a 2D frame of the scene illuminated with the projector" << std::endl;
        const auto illuminatedSceneFrame2D = camera.capture(settings2DMaxBrightness);

        std::cout << "Capture a 2D frame of scene without projector illumination" << std::endl;
        const auto nonIlluminatedSceneFrame2D = camera.capture(settings2DZeroBrightness);

        std::cout << "Locating marker in the 2D image:" << std::endl;
        const auto markerLocation = findMarker(
            projectedMarkerFrame2D,
            illuminatedSceneFrame2D,
            nonIlluminatedSceneFrame2D,
            markerResolution,
            camera.info());
        std::cout << markerLocation << std::endl;

        std::cout << "Capturing a point-cloud using Capture Assistant" << std::endl;
        const auto frame = captureWithCaptureAssistant(camera);

        std::cout << "Looking up 3D coordinate based on the marker position in the 2D image:" << std::endl;
        const auto pointsXYZ = frame.pointCloud().copyPointsXYZ();
        const auto col = markerLocation.x;
        const auto row = markerLocation.y;
        std::cout << pointsXYZ(row, col) << std::endl;

        std::cout << "Annotating the 2D image captured while projecting the marker" << std::endl;
        const auto annotatedImage = annotate(projectedMarkerFrame2D, markerLocation);

        const std::string annotatedImageFile = "ImageWithMarker.png";
        std::cout << "Saving the annotated 2D image to file: " << annotatedImageFile << std::endl;
        cv::imwrite(annotatedImageFile, annotatedImage);

        std::cout << "Done" << std::endl;
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
