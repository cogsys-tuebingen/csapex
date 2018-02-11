#!/localhome/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  groups: ~
  assignments:
    - uuid: ~
      id: 1
    - uuid: csapex::Note_0
      id: 1
    - id: 1
      uuid: csapex::ExampleImageGenerator_0
    - uuid: csapex::OutputDisplay_1
      id: 1
    - uuid: csapex::GaussianBlur_0
      id: 1
uuid_map:
  csapex::BoxBlur: 1
  csapex::ColorSegmentation: 1
  csapex::GaussianBlur: 1
  csapex::GrabCut: 1
  csapex::Note: 3
  csapex::ExampleImageGenerator: 1
  csapex::OutputDisplay: 3
  vision_plugins::ColorConvert: 1
  vision_plugins::GenericImageCombiner: 1
nodes:
  - uuid: csapex::Note_0
    label: prototype
    type: csapex::Note
    pos:
      - 21
      - -230
    color:
      - 0
      - 0
      - 0
    z: -2147483520
    minimized: false
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
        h:
          type: value
          name: h
          int: 308
        text:
          type: value
          name: text
          string: "Tutorial Series: GUI usage\nTutorial 04: Profiling\n\nThe efficiency of a synchronous dataflow graph depends on the most expensive node in the graph.\n\nFinding the bottleneck using CS::APEX is easy:\n\n- Select all nodes (CTRL + A)\n- Enable profiling by right-clicking and selecting \"start profiling\" on one of the nodes.\n\nNow try to find the bottleneck in this dataflow graph."
        w:
          type: value
          name: w
          int: 236
  - type: csapex::ExampleImageGenerator
    uuid: csapex::ExampleImageGenerator_0
    label: Input
    pos:
      - -24
      - 125
    color:
      - -1
      - -1
      - -1
    z: 142
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - type: csapex::OutputDisplay
    uuid: csapex::OutputDisplay_1
    label: Display
    pos:
      - 683
      - -58
    color:
      - -1
      - -1
      - -1
    z: 144
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - type: csapex::GaussianBlur
    uuid: csapex::GaussianBlur_0
    label: csapex::GaussianBlur_0
    pos:
      - 330
      - 58
    color:
      - -1
      - -1
      - -1
    z: 143
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        kernel:
          type: range
          name: kernel
          int: 147
          min: 1
          max: 255
          step: 2
        sigma x:
          type: range
          name: sigma x
          double: 128
          min: 0.1
          max: 128
          step: 0.1
        sigma y:
          type: range
          name: sigma y
          double: 128
          min: 0
          max: 128
          step: 0.1
      persistent_params:
        []
connections:
  - uuid: csapex::GaussianBlur_0:|:out_0
    targets:
      - csapex::OutputDisplay_1:|:in_0
    types:
      - default
  - uuid: csapex::ExampleImageGenerator_0:|:out_0
    targets:
      - csapex::GaussianBlur_0:|:in_0
    types:
      - default
adapters:
  - uuid: csapex::OutputDisplay_1
    state:
      width: 161
      height: 159