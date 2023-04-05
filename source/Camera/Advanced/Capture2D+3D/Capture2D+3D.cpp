/*
Capture 2D and then 3D using various capture strategies, optimizing for both 2D quality and 2D acquisition speed.
*/

#include <Zivid/Zivid.h>

#include <clipp.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <numeric>
#include <thread>

namespace ZividClock
{
    class StopWatch
    {
    public:
        StopWatch()
            : m_startTime(std::chrono::high_resolution_clock::now())
        {}

        double elapsed() const
        {
            const std::chrono::duration<double, std::milli> elapsed =
                std::chrono::high_resolution_clock::now() - m_startTime;
            return elapsed.count();
        }

    private:
        std::chrono::high_resolution_clock::time_point m_startTime;
    };
} // namespace ZividClock

namespace
{
    void printHeader(const std::string &label)
    {
        const size_t totalWidth = 90;
        const auto leftFill = 5;
        const auto rightFill = totalWidth - leftFill - label.size() - 2;
        std::cout << std::endl
                  << std::setw(leftFill) << std::setfill('*') << "*"
                  << " " << label << " " << std::setw(rightFill) << "*" << std::endl;
    }

    void printValue(const std::string &label, const double value)
    {
        std::cout << std::setw(75) << std::left << std::setfill(' ') << label << std::setw(7) << std::fixed
                  << std::setprecision(2) << std::right << value << "ms" << std::endl;
    }

    void printCaptureFunctionReturnTime(const double value, const std::string &prefix)
    {
        printValue(prefix + " Function Return Time", value);
    }

    void printCaptureFunctionReturnTime(const double value2d, const double value3d)
    {
        printCaptureFunctionReturnTime(value2d, "2D");
        printCaptureFunctionReturnTime(value3d, "3D");
    }

    void dummyCapture2D(Zivid::Camera &camera, const Zivid::Settings2D &settings2d)
    {
        camera.capture(settings2d);
    }

    void dummyCapture3D(Zivid::Camera &camera, const Zivid::Settings &settings)
    {
        camera.capture(settings.copyWith(Zivid::Settings::Acquisitions{ settings.acquisitions().at(0) }));
    }

    Zivid::Settings2D makeSettings2D()
    {
        return Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
            Zivid::Settings2D::Acquisition::ExposureTime{ std::chrono::microseconds{ 10000 } },
            Zivid::Settings2D::Acquisition::Aperture{ 2.8 },
            Zivid::Settings2D::Acquisition::Brightness{ 0.0 },
            Zivid::Settings2D::Acquisition::Gain{ 1.0 },
        } } };
    }


    Zivid::Settings2D setProjectorBrightness(Zivid::Settings2D &settings2D, const bool useProjector)
    {
        settings2D.acquisitions()[0].brightness() =
            Zivid::Settings2D::Acquisition::Brightness{ useProjector ? 1.0 : 0.0 };

        return settings2D;
    }

    Zivid::Settings::Acquisition acquisition3D()
    {
        return Zivid::Settings::Acquisition{
            Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 10000 } },
            Zivid::Settings::Acquisition::Aperture{ 2.8 },
            Zivid::Settings::Acquisition::Brightness{ 1.8 },
            Zivid::Settings::Acquisition::Gain{ 1.0 },
        };
    }

    Zivid::Settings makeSettings3D()
    {
        return Zivid::Settings{ Zivid::Settings::Acquisitions{ acquisition3D(), acquisition3D() },
                                Zivid::Settings::Processing::Color::Experimental::Mode::automatic };
    }

    template<typename FrameT>
    struct FrameAndCaptureTime
    {
        FrameT frame;
        double captureTime;
    };

    template<typename FrameT, typename SettingsT>
    FrameAndCaptureTime<FrameT> captureAndMeasure(Zivid::Camera &camera, const SettingsT &settings)
    {
        if(std::is_same<FrameT, Zivid::Frame>::value)
        {
            std::cout << "Begin 3D capture\n";
        }
        else if(std::is_same<FrameT, Zivid::Frame2D>::value)
        {
            std::cout << "Begin 2D capture\n";
        }
        ZividClock::StopWatch watch;
        const auto frame = camera.capture(settings);
        const auto elapsed = watch.elapsed();
        if(std::is_same<FrameT, Zivid::Frame>::value)
        {
            std::cout << "End of 3D capture\n";
        }
        else if(std::is_same<FrameT, Zivid::Frame2D>::value)
        {
            std::cout << "End of 2D capture\n";
        }
        return { std::move(frame), elapsed };
    }

    template<typename T>
    void useFrame(const T &frame);

    template<>
    void useFrame<Zivid::Frame>(const Zivid::Frame &frame)
    {
        std::cout << "Accessing and using 3D data\n";
        const ZividClock::StopWatch stopWatch;
        frame.pointCloud().copyPointsXYZ();
        frame.save("save_to_emulate_using_3D_data.zdf");
        printValue("Completed using 3D data in:", stopWatch.elapsed());
    }

    template<>
    void useFrame<Zivid::Frame2D>(const Zivid::Frame2D &frame2D)
    {
        std::cout << "Accessing and using 2D data\n";
        const ZividClock::StopWatch stopWatch;
        const auto image = frame2D.imageRGBA();
        image.save("save_to_emulate_using_2D_data.png");
        printValue("Completed using 2D data in:", stopWatch.elapsed());
    }

    void
    capture2DFirstAndThen3D(Zivid::Camera &camera, const Zivid::Settings2D &settings2D, const Zivid::Settings &settings)
    {
        dummyCapture2D(camera, settings2D);
        const auto frame2dAndCaptureTime = captureAndMeasure<Zivid::Frame2D>(camera, settings2D);
        std::cout
            << "Starting 3D capture in current thread and using 2D data in separate thread, such that the two happen in parallel"
            << std::endl;
        std::future<void> userThread =
            std::async(std::launch::async, useFrame<Zivid::Frame2D>, std::ref(frame2dAndCaptureTime.frame));
        const auto frameAndCaptureTime = captureAndMeasure<Zivid::Frame>(camera, settings);
        useFrame(frameAndCaptureTime.frame);
        std::cout << "Wait for usage of 2D frame to finish" << std::endl;
        userThread.get();
        printCaptureFunctionReturnTime(frame2dAndCaptureTime.captureTime, frameAndCaptureTime.captureTime);
    }

    void
    capture3DFirstAndThen2D(Zivid::Camera &camera, const Zivid::Settings &settings, const Zivid::Settings2D &settings2D)
    {
        dummyCapture3D(camera, settings);
        const auto frameAndCaptureTime = captureAndMeasure<Zivid::Frame>(camera, settings);
        std::cout
            << "Starting 2D capture in current thread and using 3D data in separate thread, such that the two happen in parallel"
            << std::endl;
        std::future<void> userThread =
            std::async(std::launch::async, useFrame<Zivid::Frame>, std::ref(frameAndCaptureTime.frame));
        const auto frame2dAndCaptureTime = captureAndMeasure<Zivid::Frame2D>(camera, settings2D);
        useFrame(frame2dAndCaptureTime.frame);
        std::cout << "Wait for usage of 3D frame to finish" << std::endl;
        userThread.get();
        printCaptureFunctionReturnTime(frame2dAndCaptureTime.captureTime, frameAndCaptureTime.captureTime);
    }

    void capture3DIncluding2D(Zivid::Camera &camera, const Zivid::Settings &baseSettings, const bool useProjector)
    {
        auto acquisitions = baseSettings.acquisitions().value();
        acquisitions.insert(
            acquisitions.begin(),
            acquisition3D().copyWith(Zivid::Settings::Acquisition::Brightness{ useProjector ? 1.0 : 0.0 }));
        const auto settings = baseSettings.copyWith(
            Zivid::Settings::Acquisitions{ acquisitions },
            Zivid::Settings::Processing::Color::Experimental::Mode::useFirstAcquisition);
        dummyCapture3D(camera, settings);
        const auto frameAndCaptureTime = captureAndMeasure<Zivid::Frame>(camera, settings);
        useFrame(frameAndCaptureTime.frame);
        printCaptureFunctionReturnTime(frameAndCaptureTime.captureTime, "3D");
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        bool settingsFromYML = false;

        std::vector<std::string> settingsFiles;
        auto cli =
            (clipp::option("--settings-2d-and-3d").set(settingsFromYML, true)
             & clipp::values("settings2DAnd3DFiles", settingsFiles));

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << clipp::usage_lines(cli, "ZividBenchmark", fmt) << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }

        if((settingsFiles.size() != 2) && (!settingsFiles.empty()))
        {
            throw std::runtime_error{
                "Exactly two paths must be provided, first for 2D settings, then for 3D settings."
            };
        }

        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << camera.info().toString() << std::endl;

        auto settings = (settingsFromYML) ? Zivid::Settings(settingsFiles[1]) : makeSettings3D();
        auto settings2D = (settingsFromYML) ? Zivid::Settings2D(settingsFiles[0]) : makeSettings2D();

        printHeader("2D without projector followed by 3D");
        capture2DFirstAndThen3D(camera, setProjectorBrightness(settings2D, false), settings);
        printHeader("2D with projector followed by 3D");
        capture2DFirstAndThen3D(camera, setProjectorBrightness(settings2D, true), settings);

        printHeader("3D followed by 2D without projector");
        capture3DFirstAndThen2D(camera, settings, setProjectorBrightness(settings2D, false));
        printHeader("3D followed by 2D with projector");
        capture3DFirstAndThen2D(camera, settings, setProjectorBrightness(settings2D, true));

        printHeader("3D including 2D without projector");
        capture3DIncluding2D(camera, settings, false);
        printHeader("3D including 2D with projector");
        capture3DIncluding2D(camera, settings, true);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
