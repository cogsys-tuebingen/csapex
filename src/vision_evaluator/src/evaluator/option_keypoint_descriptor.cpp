/// HEADER
#include "option_keypoint_descriptor.h"

REGISTER_OPTION(OptionKeypointDescriptor)

OptionKeypointDescriptor::OptionKeypointDescriptor()
    : Option("Keypoint Descriptor"), selection(NULL)
{
}

OptionKeypointDescriptor::~OptionKeypointDescriptor()
{
}

Option::TypePtr OptionKeypointDescriptor::createInstance(CONSTRUCTOR_MODE mode)
{
    return Option::TypePtr(new OptionKeypointDescriptor());
}

void OptionKeypointDescriptor::update(int slot)
{
    Config config = Config::getGlobal();

    if(config.getDescriptorType() == slot) {
        return;
    }

    std::cout << slot << std::endl;

    config.setDescriptorType(static_cast<Types::Descriptor::ID>(slot));
    config.replaceGlobal();

    plugin_changed();
}

void OptionKeypointDescriptor::insert(QLayout* layout)
{
    selection = new QComboBox;
    for(int k = 0; k < Types::Descriptor::COUNT; k++) {
        selection->addItem(Types::Descriptor::write(k).c_str());
    }
    layout->addWidget(selection);

    Config config = Config::getGlobal();
    selection->setCurrentIndex(config.getDescriptorType());

    QObject::connect(selection, SIGNAL(currentIndexChanged(int)), this, SLOT(update(int)));
}

void OptionKeypointDescriptor::configChanged()
{
    if(config.getDescriptorType() < 0) {
        return;
    }

    if(selection) {
        selection->setCurrentIndex(config.getDescriptorType());
    }
}
