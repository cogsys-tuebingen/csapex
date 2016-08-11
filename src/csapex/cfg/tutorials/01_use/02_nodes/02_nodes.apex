#!/home/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  groups: ~
  assignments:
    - id: 1
      uuid: ~
    - uuid: csapex::Note_0
      id: 1
    - uuid: csapex::ExampleImageGenerator_0
      id: 1
    - id: 1
      uuid: csapex::Note_1
uuid_map:
  csapex::ExampleImageGenerator: 1
  csapex::Note: 3
  csapex::OutputDisplay: 1
connections:
  []
adapters:
  []
nodes:
  - type: csapex::Note
    uuid: csapex::Note_0
    label: prototype
    pos:
      - 706
      - -228
    state:
      persistent_params:
        []
      params:
        w:
          type: value
          name: w
          int: 479
        h:
          type: value
          name: h
          int: 339
        text:
          name: text
          string: "Tutorial Series: GUI usage\nTutorial 02: Creating nodes\n\nTry creating a node of type \"OutputDisplay\"  and connect \nthe output of the available node to it.\n\nPossible ways to create nodes are:\n\n1. Node prompt dialog (CTRL + Space)\n  - Enter \"OutputDisplay\" or any combination of substrings.\n  - The search is fuzzy, which means you can enter multiple strings\n     that have to match the node you want to create. The strings are \n     also matched with other meta data of the nodes.\n  - For example try creating the OutputDisplay by entering \n      \"display qt image\"\n\n2. Context menu -> \"create node\"\n\n3. Toolpanel \"Available Nodes\""
          type: value
    flipped: false
    color:
      - 0
      - 0
      - 0
    z: -2147483555
    minimized: false
    enabled: true
  - z: 14
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
    type: csapex::ExampleImageGenerator
    uuid: csapex::ExampleImageGenerator_0
    label: Input
    pos:
      - 227
      - -97
    color:
      - -1
      - -1
      - -1
  - type: csapex::Note
    uuid: csapex::Note_1
    label: prototype
    pos:
      - 146
      - 18
    color:
      - -1
      - -1
      - -1
    z: -2147483617
    minimized: false
    enabled: true
    state:
      params:
        h:
          type: value
          name: h
          int: 111
        text:
          type: value
          name: text
          string: "The node \"Input\" produces a stream of images.\nIt has one output port named \"Image\".\n\nYou can hover over it to see the published image.\n"
        w:
          name: w
          int: 335
          type: value
      persistent_params:
        []
    flipped: false