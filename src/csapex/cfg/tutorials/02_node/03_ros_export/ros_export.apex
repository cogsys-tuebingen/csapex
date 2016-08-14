#!/home/buck/ws/apex/devel//lib/csapex/csapex_node
uuid_map:
  csapex::ExportRos: 1
  csapex::OutputDisplay: 1
  csapex::TextDisplay: 1
  csapex::tutorial::UserSwitch: 1
  vision_plugins::GenericImageCombiner: 1
  csapex::ExampleImageGenerator: 1
nodes:
  - type: csapex::ExampleImageGenerator
    uuid: csapex::ExampleImageGenerator_0
    label: csapex::ExampleImageGenerator_0
    pos:
      - -700
      - -282
    color:
      - -1
      - -1
      - -1
    z: 13
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - label: csapex::tutorial::UserSwitch_0
    type: csapex::tutorial::UserSwitch
    pos:
      - 251
      - -311
    color:
      - -1
      - -1
      - -1
    z: 2
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        selector:
          type: set
          name: selector
          txt: FIRST
          int: 0
      persistent_params:
        []
    uuid: csapex::tutorial::UserSwitch_0
  - type: vision_plugins::GenericImageCombiner
    uuid: vision_plugins::GenericImageCombiner_0
    label: vision_plugins::GenericImageCombiner_0
    pos:
      - -487
      - 62
    color:
      - -1
      - -1
      - -1
    z: 14
    minimized: false
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
        input count:
          type: value
          name: input count
          int: 1
        input names:
          type: string_list
          name: input names
          list:
            - Image
            - Image
        script:
          type: value
          name: script
          string: 255-$1
  - type: csapex::OutputDisplay
    uuid: csapex::OutputDisplay_0
    label: csapex::OutputDisplay_0
    pos:
      - 661
      - 75
    color:
      - -1
      - -1
      - -1
    z: 15
    minimized: false
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
        {}
  - type: csapex::TextDisplay
    uuid: csapex::TextDisplay_0
    label: csapex::TextDisplay_0
    pos:
      - 289
      - 327
    color:
      - -1
      - -1
      - -1
    z: 12
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::ExportRos_0
    label: csapex::ExportRos_0
    type: csapex::ExportRos
    pos:
      - 932
      - -142
    color:
      - -1
      - -1
      - -1
    z: 17
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        queue:
          type: range
          name: queue
          int: 1
          min: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: export
      persistent_params:
        []
threads:
  groups: ~
  assignments:
    - id: 1
      uuid: ~
    - id: 1
      uuid: csapex::ExampleImageGenerator_0
    - id: 1
      uuid: csapex::tutorial::UserSwitch_0
    - id: 1
      uuid: vision_plugins::GenericImageCombiner_0
    - uuid: csapex::OutputDisplay_0
      id: 1
    - uuid: csapex::TextDisplay_0
      id: 1
    - uuid: csapex::ExportRos_0
      id: 1
adapters:
  - uuid: csapex::OutputDisplay_0
    state:
      width: 512
      height: 512
connections:
  - uuid: csapex::ExampleImageGenerator_0:|:out_0
    targets:
      - csapex::tutorial::UserSwitch_0:|:in_0
      - vision_plugins::GenericImageCombiner_0:|:in_0
    types:
      - default
      - default
  - uuid: vision_plugins::GenericImageCombiner_0:|:out_0
    targets:
      - csapex::tutorial::UserSwitch_0:|:in_1
    types:
      - default
  - targets:
      - csapex::OutputDisplay_0:|:in_0
    uuid: csapex::tutorial::UserSwitch_0:|:out_0
    types:
      - default
  - uuid: csapex::tutorial::UserSwitch_0:|:out_1
    targets:
      - csapex::TextDisplay_0:|:in_0
      - csapex::ExportRos_0:|:in_0
    types:
      - default
      - default