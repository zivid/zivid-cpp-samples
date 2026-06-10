
/*
Recursively iterates through all leaf parameters in a Zivid camera’s settings.

This sample walks the entire settings tree and inspects each leaf parameter.
For every parameter, it prints:
  - the current value if explicitly set,
  - otherwise the camera’s default value,
  - and any available metadata (valid ranges or discrete allowed values)
*/

#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Zivid.h>
#include <iostream>

namespace
{
    template<typename Node>
    void printValidRange(const Zivid::CameraInfo &cameraInfo)
    {
        const auto range = Zivid::Experimental::SettingsInfo::validRange<Node>(cameraInfo);

        std::cout << "  ValidRange: [" << Node{ range.min() } << " , " << Node{ range.max() } << "]\n";
    }

    template<typename Node>
    void printValidValues(const Zivid::CameraInfo &cameraInfo)
    {
        const auto validValues = Zivid::Experimental::SettingsInfo::validValues<Node>(cameraInfo);

        std::cout << "  ValidValues: ";
        for(const auto &value : validValues)
        {
            std::cout << value << " ";
        }
        std::cout << "\n";
    }

    template<typename Node>
    void inspectLeaf(const Node &node, const Zivid::CameraInfo &cameraInfo)
    {
        using DecayedNode = std::decay_t<Node>;

        if constexpr(DecayedNode::nodeType == Zivid::DataModel::NodeType::leafValue)
        {
            std::cout << "Parameter: " << DecayedNode::path << "\n";

            if constexpr(Zivid::DataModel::IsOptional<DecayedNode>::value)
            {
                std::cout << "  CurrentValue: ";
                if(node.hasValue())
                {
                    std::cout << node.toString() << "\n";
                }
                else
                {
                    std::cout << "(unset)\n";

                    const auto defaultValue = Zivid::Experimental::SettingsInfo::defaultValue<DecayedNode>(cameraInfo);
                    std::cout << "  DefaultValue: ";
                    std::cout << defaultValue.toString() << "\n";
                }
            }
            else
            {
                std::cout << "  CurrentValue: " << node.toString() << "\n";
            }

            constexpr bool hasRange = Zivid::DataModel::HasValidRange<DecayedNode>::value;
            constexpr bool hasValues = Zivid::DataModel::HasValidValues<DecayedNode>::value;

            if constexpr(hasRange)
            {
                printValidRange<DecayedNode>(cameraInfo);
            }

            if constexpr(hasValues)
            {
                printValidValues<DecayedNode>(cameraInfo);
            }

            if constexpr(!hasRange && !hasValues)
            {
                std::cout << "  No predefined valid range or discrete values.\n";
            }

            std::cout << "\n";
        }
    }

    // Recursively traverses nested data model leaves and lists
    template<typename Node, typename LeafVisitor>
    void traverseLeavesIntoNestedDataModelsAndLists(Node &node, const LeafVisitor &leafVisitor)
    {
        using DecayedNode = std::decay_t<Node>;

        if constexpr(
            DecayedNode::nodeType == Zivid::DataModel::NodeType::group
            || DecayedNode::nodeType == Zivid::DataModel::NodeType::leafDataModelList)
        {
            std::forward<Node>(node).forEach([&](auto &&childNode) {
                traverseLeavesIntoNestedDataModelsAndLists(std::forward<decltype(childNode)>(childNode), leafVisitor);
            });
        }
        else if constexpr(
            DecayedNode::nodeType == Zivid::DataModel::NodeType::leafValue
            && Zivid::DataModel::IsNestedDataModelLeaf<DecayedNode>::value)
        {
            leafVisitor(node);

            if constexpr(Zivid::DataModel::IsOptional<DecayedNode>::value)
            {
                if(node.hasValue())
                {
                    traverseLeavesIntoNestedDataModelsAndLists(node.value(), leafVisitor);
                }
            }
            else
            {
                traverseLeavesIntoNestedDataModelsAndLists(node.value(), leafVisitor);
            }
        }
        else
        {
            leafVisitor(std::forward<Node>(node));
        }
    }

    // Serves as the entry point to traverse a Settings object and print all leaves
    template<typename SettingsType>
    void traverseAndPrint(const SettingsType &settings, const Zivid::CameraInfo &cameraInfo)
    {
        traverseLeavesIntoNestedDataModelsAndLists(settings, [&](const auto &node) { inspectLeaf(node, cameraInfo); });
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application app;

        std::cout << "Connecting to camera...\n";
        auto camera = app.connectCamera();
        const auto cameraInfo = camera.info();

        std::cout << "Camera model: " << cameraInfo.modelName() << "\n";
        std::cout << "Serial number: " << cameraInfo.serialNumber() << "\n";

        // Default 2D Settings (mostly set)
        auto defaultSettings2DWithAcquisition =
            Zivid::Experimental::SettingsInfo::defaultValue<Zivid::Settings2D>(cameraInfo)
                .copyWith(
                    Zivid::Settings2D::Acquisitions{
                        Zivid::Experimental::SettingsInfo::defaultValue<Zivid::Settings2D::Acquisition>(cameraInfo) });

        // Minimal 3D Settings (mostly unset)
        Zivid::Settings settings;
        settings.set(Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} });
        settings.set(Zivid::Settings::Color{ defaultSettings2DWithAcquisition });

        std::cout << "---- Traversing Settings ----\n";
        traverseAndPrint(settings, cameraInfo);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
