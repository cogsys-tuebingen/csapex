/// HEADER
#include "option_keypoint_extractor.h"

REGISTER_OPTION(OptionKeypointExtractor)

OptionKeypointExtractor::OptionKeypointExtractor()
    : Option("Keypoint Extractor"), selection(NULL), threshold(NULL)
{
}

OptionKeypointExtractor::~OptionKeypointExtractor()
{
}

Option::TypePtr OptionKeypointExtractor::createInstance(CONSTRUCTOR_MODE mode)
{
    return Option::TypePtr(new OptionKeypointExtractor());
}

void OptionKeypointExtractor::update_type(int slot)
{
    Config current = Config::getGlobal();
    current.setKeypointType(static_cast<Types::Keypoint::ID>(slot));
    current.replaceGlobal();

    plugin_changed();
}

void OptionKeypointExtractor::update_threshold(int t)
{
    Config current = Config::getGlobal();
    current.extractor_threshold = t;
    current.replaceGlobal();

    plugin_changed();
}

void OptionKeypointExtractor::insert(QLayout* layout)
{
    selection = new QComboBox;
    for(int k = 0; k < Types::Keypoint::COUNT; k++) {
        selection->addItem(Types::Keypoint::write(k).c_str());
    }
    layout->addWidget(selection);

    Config config = Config::getGlobal();
    selection->setCurrentIndex(config.getKeypointType());

    QObject::connect(selection, SIGNAL(currentIndexChanged(int)), this, SLOT(update_type(int)));


    threshold = new QSlider(Qt::Horizontal);
    threshold->setMinimum(1);
    threshold->setMaximum(200);
    threshold->setValue(config.extractor_threshold);

    layout->addWidget(threshold);

    QObject::connect(threshold, SIGNAL(valueChanged(int)), this, SLOT(update_threshold(int)));
}

void OptionKeypointExtractor::configChanged()
{
    if(config.getKeypointType() < 0) {
        return;
    }

    if(selection != NULL) {
        selection->setCurrentIndex(config.getKeypointType());
    }
}

