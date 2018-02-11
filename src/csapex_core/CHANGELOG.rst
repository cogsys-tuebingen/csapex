^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package csapex
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.4 (2016-11-24)
------------------
* removed debug ouputs; builds on kinetic
* ported to kinetic
* use tempoary file name based on pid to get unique file handles
* fixed cmake installing
* suppress plugin.xml files that do not exist
* fixed race-condition when multiple instances want to persist settings
* added observer class to generalize reactive observing behaviour
* Contributors: buck, robot

0.9.3 (2016-11-19)
------------------
* Fixed cmake problems on the ROS build farm, now that prerelease is possible
  Added all dependencies
* Contributors: buck

0.9.2 (2016-11-18)
------------------
* added missing dependencies for qt5
* Contributors: buck

0.9.1 (2016-11-18)
------------------
* fixed error in un-/grouping
* fixed parameter context menu offset
* Merge remote-tracking branch 'origin/compression' into devel
* assigning threads works again
* refactored connection command handling; fixes deletion segfaults
* bugfixes and extensions messeage vector compression works
* compression
* allowing message template subclasses to call a helper for cloning
* added iostream includes sometimes necessary
* removed spam
* extracted activity modifier
* moved console_bridge use to csapex_ros
* extracted context menu logic from graphview
* improved node creation script to allow SHARED libraries
* Merge branch 'devel' of gitlab.cs.uni-tuebingen.de:csapex/csapex into devel
* dropping NoMessage tokens, when they are sent to a dead-end sub graph; bugfixes
* using transaction style to avoid graph analysis for larger, composed changes
* fixed some reset issues
* refactored memory management of connectables
* began moving logic to vertex
* began extracting vertex from nodehandle and edge from connection
* improved graph characteristic calculation; basic NoMessage pruning implemented
* split graph in two
* graph loading speed up via bugfix
* thread assignment is now recursive: sub graphs are also assigned, if they currently belong to the same group
* updating snippet list, when a new snippet is added
* reimplemented snippets; snippets can be added the same way as nodes
* lots of slot-related bug fixes
* added log level setter
* added a mute command to suppress a node's output
* fixed crash with sticky nodes
* simplyfied nested processing and fixed some bugs
* moved pipelining / serial from connection to node
* began refactoring connections
* changed parameterizable to use weak_ptr instead of raw pointers
* fixed bug with focus on box dialog
* added frequency display that shows how often a node runs per second
* added another way to specify conditional parameters
* added a notification widget that shows errors from nodes; clicking notifications jumps to the source node
* errors are also shown in the status bar
* updated node creation script
* improved c++11 detection
* cleaned up some transitive includes for plugin nodes
* cleaned up object creation and main function
* added slot callback with pointer to the slot itself
* restoring thread assignment works
* activity timeline shows interval steps
* activity timeline now shows node *activity*
* switched profiling signals to emit intervals instead of timestamps
* new connect function that directly takes a member function pointer
* cleaned up activity timeline; improved visual appearance
* suppress warning
* showing function signature in assertion
* began refactoring timeline: added recording / paused state
* fixed set parameter initialization bug
* improved profiling widget: hovered blocks are highlighted and give a tooltip; added profiler for graphio
* added more guards
* added lookup for mapped parameters
* fixed a bug in the scheduler, where newly created thread groups were not started
* Fixes #82: Stop auto scrolling at the border, if hovering an item
* Fixes #85: Fixed assertion error when resetting interactive nodes
* cleaned up iteration; some bugs fixed
* added framework for subgraph iteration
* implemented another generic vector representation for arbitrary message contents
* Fixed bug in Timer, where the first measurement could be invalid
* unified singleton shutdown
* save vector message as one file
* removed spam
* fixed async problem in nested graphs that are sinks
* Fixes #94: Fixes profiling widget connection not being disconnected
* Fixes #96: On my machine I can no longer cause the multi-clone problem
* Fixes #97: Reset clears the GUI completely
* Fixes #81: Subgraph scheduling works more robustly
* reimplemented subgraph scheduling
* initializing label text from parameter
* Fixes #93: Disabled Slots' callbacks are no longer called; Disabled non-active ports are visually marked again
* implemented a new output parameter: text -> is displayed on a QLabel
* added a hook for loading additional information from manifest xml file
* suppressed warnings when diverging streams are combined
* fixed box dialog issue when one keyword is '.'
* Fixes #91: limitting the step size of range parameters
* extended debug profiler for designer scene
* added vector support for non-default constructable values
* Fixes #92: buildfix
* fixed uninitialized vector message
* various bugs fixed; loosened some assertions
* don't block when adding a new connection
* buildfix on linux
* better cmake support for non-catkin plugins
* windows specific implementations added; added windows icon; fixed some
  problems with the singletons on windows
* Merge remote-tracking branch 'origin/devel' into windows_port
* Merge branch 'devel' of gitlab.cs.uni-tuebingen.de:csapex/csapex into devel
* fixed multiinput bug vector messages
* added an elapsed function to timer
* always displaying menu bar: fixes shortcut ambiguity problems
* reimplemented move command without gui specifics
* Renaming forwarding connectors now also renames their counterparts
* Fixes #80: Sorting UUIDs when requesting them from transitions
* merged window specific macros
* added more tutorials
* Merge branch 'windows_port' into devel
* variadic methods
* builds on windows
* Added two further tutorials; Fixed the Ubuntu desktop file generation; Bugfix in node color loading
* creating issues now lets reporters select the target website
* Merge branch 'devel' of gitlab.cs.uni-tuebingen.de:csapex/csapex into devel
  Conflicts:
  src/csapex/src/view/node/box.cpp
* added interactive tutorial support; added two basic tutorials
* updated script collection
* changing box stylesheet only when necessary
* should fix the shortcut problem
* added README and removed unnecessary dependency on console_bridge
* enforcing new shortcuts
* bump to 0.9.0
* removed dynamic ports completely
* made cloning of vectors a deep copy as intended
* moved generic vector into main repository
* eliminated VectorMessage
* removed warning for missing adapter - also prints for non-adapted nodes
* Merge branch 'unstable' into devel
* moved nested profiling classes to separate files; extracted profiling library
* implemented debug profiling
* refactored profiling widget, made it independent of the model
* moved profiling classes
* extracted profiler class
* inverted profiling data flow
* added context menu entry to en-/disable nodes
* typing in a text box no longer instantly changes the parameter
* improved parameter context menu usage
* dialogs no longer fall to the back; added a menu entry for node creation
* implemented node search
* added tearDown to node interface
* allowing to create new emtpy subgraphs
* removed graph levels
* renamed message traits to token traits
* immediate ticking no longer starves other nodes
* Recovery is now on a timer instead of each individual change
* Fixes #86: Profiling widget no longer segfaults on node deletion
* Fixes 87: Fixed deserialization bug for fulcrums
* Fixes #88: Marker messages are no longer set as output types
* fixed message preview crashes
* hiding global ports when the graph is empty
* exit slot added
* added more helper functions to create slots / events
* Fixes #77: Mapping names to valid ros names for comparison
* fixed maximizing boxes not working
* Fixes #79: Reimplemented clone-by-drag: Works the same as copy-paste now -> allows cloning multiple nodes at once
* Fixes #78: When running from a terminal, the StreamInterceptor now correctly destructs
* Fixes #76: Fixed stale pointer
* Fixes #74: Bugfix
* Fixes #72: Bug in graph -> find
* finished implementing node type change dialog
* shutdown bug fixed
* suggest labels when creating ports
* preserve active connections when deleting / restoring them
* plugin loader fix
* continued rewiring - displays old and new states
* refactored graph view dependencies; work toward rewiring dialog
* added searchable properties to nodes
* renaming connectors
* template slot
* allow deactivation of connections in gui
* saving variadic port labels
* refactored variadic port creation to support more complex setup
* added labeling for internal ports; tweaked layouts
* removed spam
* finished making signals typed; fixed activity issues; further bugfixes and refactorings
* correctly forwarding activity into subgraphs
* added internal slots
* allowing core plugins to modify the graph
* made variadic io accessible; bugfix
* made linear fulcrums the default; bugfix
* added option to make parameters hidden
* removed debug info; bugfix
* split token into data part and pure token
* moved active state into nodestate
* Using root graph in the same way as a nested graph;
  various refactorings to support internal events
* offset port labels for events and slots
* implemented basic activity mechanic
* refactored ticking
* preview widget improvements -> rendering image not in GUI thread
* began simplifying data flow communication
* removed establishing of connections
* removed signal specific code
* Merge branch 'unstable' of gitlab.cs.uni-tuebingen.de:csapex/csapex into unstable
* added typed signals
* Renamed ConnectionType to Token
* renamed Trigger to Event
* allow searching for nodes by label
* using images for ports to better distiguish them
* replaced gray meta port with image
* unified variadic creation; variadic nodes are now undoable
* simplified subgraph execution model
* minimized the amount of forwarding ports created by grouping
* grouping + ungrouping now also works with signals
* icon for parameters with tooltip added
* graph varidics are now commands
* both connection types can be connected to variadic ports
* starting execution after gui is loaded
* using variadic io for graph
* towards merging graph + variadic
* renamed pass out connector to add vadiadic connector
* implemented signal support for subgraphs
* bugfix in combobox
* apex assert added
* refactored io enabling
* inverted inheritance hierarchy for variadic io
* towards using meta port for variadic nodes
* message preview is its own window now
* changed fonts to remove text render artefacts; hiding port meta info when the port is not visible
* subgraphs are now deleted with their parent node
* disable grouping buttons in the menu when they are unusable
* resetting now also works for subgraphs
* moved "move connection" into command factory; added debug information display for graph
* began work in meta port
* Merge branch 'grouping' into devel
* publishing bool parameters
* subgraphs handle correctly
* fixed zooming and panning problems
* removed spam
* made renaming a command
* allow renaming tabs
* sources and sinks now work on this machine
* continued nesting; sources still buggy
* tabs are now updated, when nodes are renamed
* ungrouping works; fixed serialization problems for graphs
* subgraphs can now be deleted and restored
* showing "ungroup" for graph nodes
* check conditions after updating parameters
* repaint background when boxes are added or removed
* refactored absolute uuids; fixed some subgraph issues; added shortcut for subgraphing; added preliminary ungroup button
* began transition to absolute uuids
* fixed a few bugs in the new parameter adapters
* fixed bug in copy paste where connections weren't copied
* forgot to add the new files
* done refactoring parameter adapters
* extracted value parameter adapter
* refactored param setting to command
* broken
* fixed dnd issues
* added marker message to signal the end of stream
* Fixes #66: Segfault fixed when cloning nodes with interactive parameters
* Fixes #54: Highlighting connections to currently selected nodes;
  Connections are now renderered slightly transparent by default
* Fixes #68: Fixed some problems with temporary connections to the message preview widget
* added option for conditional ticking
* added sync slot; fixed dangling pointers in adapters causing segfaults regularly
* set parameter += getter for values
* Fixes #67: cleaned up reset
* message provider += restart
* fixed port lookup segfault
* fixed angle param problem
* loading plugins lazily when they are used
* moved "resend" to base message provider
* fixed submenu not being visible in node context
* missing return
* implemented sticky nodes; allowing users to choose colors for nodes
* context menu for angle parameter; fixes
* bugfixes; priority
* added widget to display relayed ports
* implemented nesting more generically
* fixed fulcrum problems
* added absolute uuid class
* refactored commands to work for sub graphs
* refactored UUID
* renaming
* fixed cache problem with preview widget - invisible connectors
* fixed undo of AddNode not working
* eradicated widget controller; extracted designer options class
* auto resize scene when scrolling
* extracted clipboard commonalities
* removed more parts of widget controller
* sub graphs can now be opened and closed
* eliminated most of widget controller; added tabbed view for multiple graphs; lots of bug fixes; clean up
* creating scene inside designer
* renamed DesignerView to GraphView
* nested uuid lookup
* refactored internals
* implemented copy & paste
* one layer nesting works
* refactored UUID maintanance into separate class
* quick bug fix
* lots of small gui improvements; allowing resizable node adapters
* executors are now composable too
* graph acts as a node
* cleaner initial view
* fixed initially wrong style; improved move performance
* removed unloading / reloading parts since that cannot be achieved realistically
* fixed shutdown segfault
* switched from boost signal to custom slim signals
* further reduced include load
* fixed some shutdown problems
* extracted message implemenations
* removed a lot of boost
* some boost cleanup
* graph uses node handle, not not worker; render generic messages
* node worker no longer derives from node handle; added fast delegates implementation
* some clang warnings removed
* cleaned up missing overrides, builds in clang
* renamed GraphWorker to GraphFacade
* using node handle instead of node worker where possible
* test driven refactorings
* pulled up parts of nodeworker
* added debug mode; fixed bug in default thread group
* extracted exception handling logic
  fixed some bugs
* extracted node listing
* changed hard assertions -> allowing bug reporting
* workaround for qt5 point mapping bug #14090
* implemented config recovery
* undo / redo: show what will be un/redone
* Fixes #60: done refactoring preview
* more stable connecting
* proper display handling using signals
* preview works on connections
* non init bug
* preview works on inputs too
* preview works; establishing connections should be more reliable
* using message framework for preview
* Merge branch 'devel' into refactoring
  Conflicts:
  src/csapex/include/csapex/manager/message_renderer_manager.h
  src/csapex/src/manager/message_renderer_manager.cpp
* fixed shutdown segfault
* fixes
* extracted more functionality from node worker
* moved part of the  process logic from node worker into input transition
* refactored sequence numbers
* fixed shutdown segfault
* more refactorings
* refactored commands
* various
* refactored inputs and outputs
* core is now completely Qt-free
* moved view classes to view dir; refactored stream interceptor
* fixed more merge errors...
* buildfix - merge conflict overlooked
* implemented ticking without QTimer
* for now: explicitly destroy the graph
* fixed auto scrolling bugs
* implemented builder pattern for parameters
* merged back utils_param
* merged back utils_param
* split forward delcarations into packages
* removed command creating from core models into command factory
* removed old test files
* no more view/ includes in core lib
* moved designer io to view
* removed register script for node adapters
* refactored drag io
* moved dragio
* more restructuring
* (2) restructuring
* structured view directory
* moved qxt stuff into external
* Removed unnecessary assertion.
* copied over the few dependencies from utils_qt
* fixed angle parameter deserialization bug
* Refactored parameter I/O ports
* Mode enable / disable node a command
* State bug fixed by making Connections initially "done"
* Merge remote-tracking branch 'origin/bugfix' into devel
* more informative splash screen implemented
* isConnected was bricked
* supporting angle parameter
* quick fix
* Implemented Stepping; Fixed stupid bug (uninitialized variable)
* better state renderering; misc
* ticking now requires to derive from TickableNode
* began refactoring ticking
* removed old api function
* fixed threads not being displayed correctly
* replaced resize grip with better visible image
* implemented middle mouse button scrolling
* made scroll border smaller
* made boxes resizable
* made profiling widget resizable
* bugfix for deserializing thread assignments
* fixed inital pausing not working anymore; added reset and export to profiling widget
* Added support for more parameter types
* Stability changes
* misc fixes
* some stability problems due to dangling pointers fixed
* serialization for abstract messages
* Refactored serialization and ROS support
* towards better serialization
* began serialization cleanup
* bumped up version to 0.8.0 (alpha)
* refactored serialization mapping
* fixed cloning not working
* removed debug label
* refactored ros remapping support
* more graceful error handling than SIGABRT...
* allowing continuation style processing nodes (e.g. interactive node)
* fixed reset
* removed debug rendering of unestablished endpoints
* build fixes for clang
* cleanup cmake
* extracted legacy unit testing
* refactored graph handling and state reset (clearing blocking edges)
* refactored pausing
* fixed context menues
* upped the version; info is only generated when updated -> no needless relinking on cmake...
* fixed connecting asynchronous sequences
* removed spam
* more race conditions fixed
* fixed one connection problem
* using weakptrs for NodeAdapters
* QSharedPointer<QImage> -> QImage since QImage is already sharing resources
* various improvements
  * implemented error handling for non qt threads
  * fsm state errors fixed
  * slots are triggered in their corresponding runners
* fixed shutdown problem
* node adapters are useing weakptr of node worker
* fixed a state problem for unconnected outputs
* bug in connection highlighting fixed
* removed qt signals from node worker
* fixed empty minimap artifact
* preview window fixed
* fixed pipelining
* grid now instant repainted once setting changes
* reenabled pipelining
* refactored node constructor to implement builder pattern; sorting tags during node construction
* one shutdown deadlock fixed
* generic node construction is now more flexible
* refactored generic node, no more macro magic
* most of the threading code is now extracted from node worker
* switched to shared ptrs
* extracted thread group
* began scheduling refactoring
* moved more signals from qt to boost
* moved profiling signals from qt to boost
* refactorings
* made connectable independant of qt; there are some issues that will be resolved when nodworker is no longer a qobject
* fixed temporary parameters not being deserialized correctly
* deleting threads on node deletion
* removing temporary parameters now triggers the signal
* split connection enabled
* clearing blocking connections works again
* various fixes
* potential segfault
* deleting connections is now done once nodes are idle
* replaced qt foreach with c++11 foreach
* fixed shutdown segfault / locking problem
* made thread pool independent of qt
* made core independent of qt
* moved filter proxy model to view, where it belongs
* made graph independent of qt
* made graphio independent of qt
* made command dispatcher independent of qt
* made fulcrum independent of qt
* made connection and fulcrum independent of QObject
* unnecessary slot removed
* potential segfault fixed
* right click no longer deletes connections
* locking plugin handling
* explicit repainting on error, now necessary because of fewer refreshes
* no longer disable io on error
* parameter io is now also done using transitions
* generalized process
* faster redrawing after stylesheet changed
* node finder can no longer fall to background; node list is hidden while moving the finder
* fixed context menu associated to the selected instead of clicked box
* added serialization manager
* reduced needless redrawing
* using node label as prefix for output streams
* generation of debug info while compiling
* using multipart message for determination of stream end
* multiplexing works again
* visualization of connection level
* declutter
* deleting connections also works again
* adding connections works with state machine
* selective connecting hiding
* not using native dialogs
* pausing graph when opening a file
* screenshot dialog added
* review version
* more refactorings; still not complete functionality
* refactored to current interface
* model works multithreaded
* multiplex works in single thread
* demultiplexing works single threaded
* nearly working in threadless
* dead end?
* began separation of input and output
* towards dynamic io
* moved input + output templates into separate accessor namespace
* Merge branch 'clang';
* nodeworker reduction; recursive deadlock fixed
* switched generated header for compiler flags
* adapters work again
* clang works now (most of the time)
* fixed headless not working
* almost works with clang, node adapters still buggy
* Merge branch 'devel'
* border for selected boxes is now blue
* fixed self-deadlocking in interactive nodes
* compiles with clang; plugins cannot be loaded
* black is green
* bumped up the version
* shortcuts work again
* disabled boxes now painted correctly
* renamed stamp
* fixed stop race condition
* profiling widget refactored - now thread safe
* Fixes #39: Context menu now considers every selected node.
* boost bind -> std bind + c++11 lambda
* segfault "fix"
* boost stuff -> std
* various refactorings
* removed pointers to std::mutex
* replaced QMutex with std::mutex
* replaced some qt stuff
* improved error visualization: no longer overlayed
* drawing background instead of loading an image
* switched to std::shared_ptr
* pluginbase not needed
* some bugs fixed for eva tests
* began giving credit :-)
* generic node works again
* NULL -> nullptr
* cloning messages is no longer necessary -> they're now const anyway
* getMessage returns a const object
* more css control; began simplifying ui
* default signals: tick done, process done
* added active slots
  active slots can even be triggered, when a node is disabled
* c++11
* don't trigger slots when nodes are disabled
* minimap allows zooming
* added a minimap; added more icons
* fixed cloning bug
* two click connection forming
* hot borders for scrolling
* when clicking a box, the box is brought to the front
* temporary parameters can be removed + misc changes
* timeline can be reset; misc. improvements
* only show timeline for profiled nodes
* moved profiling flag to worker
* basic activity timeline added
* reloading plugins: WARNING: OS may decide to *not* reload a library...
* added plugin menu; plugins can selectively be ignored
* less dependencies
* Fixes #48: Progressbars are possible using OutputProgressParameters (see Delay)
* commands for thread control
* minimizing is now a command; improved rendering of minimized / hidden port's connections
* flipping a box also flips parameter connectors
* flipping boxes is now a command; prepared further commands
* reduced dependencies slightly
* moved node state to worker
* moved type to worker
* moved uuid from node to nodeworker
* message renderer can supply parameters
* forcing message publishing for observed outputs
* tooltip on outputs now shows messages, if a suitable message renderer exists
* introduced message renderer
* timer problem solved
* moving boxes is undoable again
* small yaml io improvements
* repaint profiling widget on tick
* fixed timer problem on tick
* handling temporary parameter callbacks
* double value now has larger range
* fixed parameter context menu being placed at the wrong position
* slots and triggers are now n:m
* signals are now movable
* fixed connection drawing
* automatically create slot+trigger for trigger parameters
* fixing mouse event bug
* MessageProvider for apexm messages
* Merge branch 'master' of gitlab.cs.uni-tuebingen.de:csapex/csapex
* pushed pluginlib dependency to csapex_ros
* qt5 port
* slot handling in worker thread
* prepared debug output stream
* immediate ticking works again
* Merge branch 'master' of gitlab.cs.uni-tuebingen.de:csapex/csapex
* improved picker; fixed connection drawing bug
* WidgetPicker; Several fixes and improvements
* changes for eva
* slots now have callbacks
* slots are triggered synchronously
* correctly serialize signals
* correctly display triggers
* preparation for signals
* extended message provider to support multi-message providers
* various refactorings, fixes and error handling
* fixed memory leak
* fixed memory leak
* removed special color for active nodes
* removed has_msg\_ map
* parameters are connectable again
* persistent parameters
* node adapters now take node workers
* refactored generic value message -> now transparent
* defaulting CMAKE_BUILD_TYPE to RelWithDebInfo
  can be changed via cmake parameter, e.g.:
  -DCMAKE_BUILD_TYPE:=Debug
* fixed bug "Uncatched exception:cannot change into directory"
* added panic (for clients)
* Merge branch 'master' of arnie:/home/robot/ws/robotle/src/csapex
* fixes on arnie
* paused
* added --fatal_exceptions mode
* checking parameter conditions on tick now
* bug fixes
* fixed race condition
* no label on node clone
* Version 0.4
* loads of bugfixes and improvements
* refactored threading seems to work
* refactoring node <-> worker relationship
* refactored utils_param
* Merge branch 'master' of gitlab.cs.uni-tuebingen.de:csapex/csapex
* immediate is now immediate
* Merge branch 'master' of gitlab.cs.uni-tuebingen.de:csapex/csapex
* toward complete yaml export ability
* refactored yaml
* fixed shutdown problem
* Fixed behaviour of optional inputs.
* better support for message providers
* extracted node adapter factory from node factory
* cleaned node factory a bit
* settings now mostly clean
* settings uses more parameter stuff
* removed graphics artifact on start
* better handling for optional inputs
* 3 step core plugin init
* ros compatibility + missing file
* refactored message YAML I/O
* refactored timing + yaml
* began refactoring yaml
* fixed some rendering bugs
* fixed a bug where disabled nodes are not correctly initialized
* generic node factory
* dynamic node experiments; not compiling
* began unit testing
* box: no more dispatcher
* removed several unnecessary dependencies
* box manager -> node factory
* cont. graph worker
* refactoring
* new api in creation script
* fixed bug in node creation script
* stop-
* cont. cleaning node interface - less include dependencies
* cont. cleaning node interface - some renaming
* cont. cleaning node interface; fixed connection rendering
* fixed drag io
* smooth zoom; focus problem fixed
* refactorings + intro text for empty boards
* cleaning node interface
* renamed connectors to better fit their purpose
* refactored messages a little
* load tags from xml
* load tags from xml files
* reimplemented icons
* reduced dependency on pluginlib; sped up startup
* Merge branch 'master' of gitlab.cs.uni-tuebingen.de:csapex/csapex
* renamed param<..> to readParameter<..> to avoid name collision with the param namespace
* refactored profiling timers
* fixxes on slieders
* removed dependency on utils_plugin
* improved range parameter updates
* range update fix
* no more yaml flow
* refactored message stuff
* renamed generic messages
* tag -> shared_ptr; release build
* refactored constness
* improved node creation script
* refactored message factory; fixed arrow rendering
* removed initial size
* bugfix: empty scene
* improved clone handling
* clone state when cloning box
* help center += node information help
* help center; about
* refactored node stats
* parameter descriptions supported
* script to create nodes easily
* bugfix with box selection handling
* drawing tweak
* towards ros independence (except catkin)
* improved visuals; rendering box overlay when zoomed out
* improved connection rendering
* more fixes
* some fixes + cleanup
* updated style sheet
* merged with opengl view; warning minimized
* relabeling boxes works again
* gridlock works again
* connection context menu readded
* fulcrum context menu
* implemented fulcrum commands
* fulcrum can be moved on creation
* extracted fulcrum; fulcrums work again
* fixed schema
* improved profiling visuals + bugfix
* profiling += mean, std_dev, legend
* oscilloscope style
* profiling works again
* some bugs fixed; improved mouse i/o; reintroduced box cloning
* usability
* suppressing warnings
* nicer grid
* moving boxes fixed; some selection stuff fixed
* began gl view; not feature complete
* Fixes #26: Pausing and Deleting no longer interfere
* Fixes #29: exception handling for Node::process refactored;
  only params with state will be initialized;
* stability
* began extracting parameter generation
* Fixes #27: Improved parameter and adapter serialization
* node adapter settings are correctly saved again
* speedup start
* Fixes #31: improved default adapter parameter hiding
* allowing to have groups closed on default
* shutdown hooks; bugfixes
* began node modifier; added multi type edge
* Fixes #25: If an output doesn't send any messages, a NoMessage is sent
* refactored ui to be persistent, using dock widgets
* began ui cleanup
* accidentally commited debug statement
* multiple fixes
* Fixes #23: fixed async malfunction
* multiple fixes; improved sync
* sync seems to work now
* syncronization works in one component
* added a button to clear blocked connectors
* moved thread logic to worker
* fixed race condition on stoping nodes before deleting them
* refactored parameter management in nodes
* fixed component labeling
* fixed setup css
* began refactoring synchronization
* improved ui generation
* Fixes #19: Finally eliminated every trace of BoxedObject
* made state accessible to children
* reduced mocing
* improved temp parameters
* bugfix in macro
* refactored tick per node
* bugfix with path parameters
* moved some responsibilities
* removed some deprecated stuff
* Fixes #2: bugfix with managed inputs
* fixes #8: improved path parameters
* output streams -> debug view
* custom out streams per node
* improved debug info
* resolved yaml compability problem
* yaml-cpp is not necessary anymore
* unnecessary deps
* merge + fix
* merge
* switch to yaml wrapper
* continued mvc
* port no longer accessible from connectable
* central, persistent settings
* Merge branch 'devel' of gitlab.cs.uni-tuebingen.de:csapex/csapex into devel
* interval slider bug
* filesystem3
* box <-> node independence done
* box <-> node independence (2)
* box <-> node independence (1)
* box no longer accessible from node
* extracted classes
* graph is view independent
* continued eliminating node -> box dependency (2nd)
* continued eliminating node -> box dependency
* began eliminating node -> box dependency
* parameters can now be made connectable
* began parameter context menu
* ported passthrough; scope change for interval parameters
* added ros interface; command for pause/unpause added
* ros launch support; graceful shutdown
* improved signal handling
* improved headless mode
* missing signal handling
* moved param connectors into node
* eliminated unnecessary messages
* text display uses yaml export
* parameters are connectable - sloppy implementation
* unified number inputs
* eliminated doublemessage
* ported double input
* moving implementation of node adapter to default node adapter
* began splitting nodes into model and view
* refactored parameter display
* towards new api
* various fixes and refactorings
* fixed setup
* improved conditional parameters
* Fixes #21
* some bugs fixed and nodes ported
* fixes and refactorings to node
* Fixes #18
* more settings
* merge + extracted settings
* delete button
* Fixes #5: added specialized int slider that allows stepsizes != 1
* renamed allConnectorsArrived() to process()
* Fixes #14
* Fixes #17: improved qdrag usage
* removed template stuff
* more dependency injection
* refactored DragIO; some bugfixes
* several fixes
* Fixes #12: Spinbox change triggers callbacks again
* conditional parameters implemented
* implemented grid locking
* refactored image encodings; some misc. bugfixes
* Fixes #11: Save as button added
* missing plugins now no longer kept
* Merge branch 'master' of gitlab.cs.uni-tuebingen.de:csapex/csapex
* bitset support
* fix
* streamlined parameter groups
* Merge branch 'master' of gitlab:csapex/csapex
* Fixes #4: DoubleSliders shouldn't produce feedback anymore
* parameter grouping implemented
* Fixes #3 Tags are created when they don't exist
* better grid (100x100)
* added pause and toolbar
* quit shortcut
* Merge branch 'master' of gitlab.cs.uni-tuebingen.de:csapex/csapex
  Conflicts:
  src/csapex/src/utility/qt_helper.cpp
  src/csapex/src/view/node_adapter.cpp
* began help menu
* image to point cloud nodes
* support for interval parameters
* several small bugfixes
* some performance issues fixed
* boxes can be flipped
* foreach fix
* cleanup; bugfixes; chrashed fixed; foreach support cont.
* label bug fixed
* several fixes for synch, refactored keypoint renderer
* began improving profiling
* added support for color parameters
* quickfix
* using new parameter disable feature
* generic vector message added
* simplified vector generation
* removed multi; fixed (a)sync
* synchronizing (async too) should work
* various improvements in sync management, still some bugs
* various fixes and improvements
* began highways
* multiple fixes / improvements
* ensuring uuids are unique
* refactored UUIDs
* reduced mocing
* improved scrolling -> locking interation
* serialization of the view position
* ros msg type complete
* began refactoring msgs
* misc fixes
* improved focus handling
* misc. improvements and fixes
* renamed connector.h -> connecable.h
* split connector into port continued
* began: connector -> port
* Merge branch 'tmp' of u-172-c010:/localhome/buck/ws/apex/src/csapex into devel
* refactored qt helper
* fixed set parameter not updating correctly
* added hough circle; many performance issues fixed
* prompt for adding nodes done
* completer - not complete
* path parameter
* added roi message; vector message; vj detector
* set params support
* stupid bug fixed
* migration to new utils_param
* split main csapex lib in two to speed up compilation times
* fulcrum types
* prepared different fulcrum types
* added a new debug view for the undo/redo stacks
* program parameter for startup config
* began removing dependencies on utils
* quick fix
* first generic node
* fixed connection bug where incompatible types can be connected
* switch to Node as top level base class
* overlay uses css
* more css; fixed loading / resetting
* checkboxes fixed; connectorless boxes rendered correctly
* no more stylesheets in .ui files
* began extracting stylesheets
* added a debug view to the ui
* mostly everything refactored
* reverted completely to pluginlib to speedup loading times
* moved box to view module
* almost everything works again
* renamed node commands
* refactoring continued
* began making graph independant of box (instead using node)
* aliminated old registration method
* proper splash screen
* more refactorings in box
* more encapsulation
* made box private in boxed object
* boxedobject is now a simple helper class
* renamed "Displayable" to "ErrorState" - less misleading
* box no longer depends on boxedobject
* removed accessor for nested object
* centralized plugin registration macro for easier modifiability
* segfault fixed
* some compiling speedup
* more refactorings
* fixed segfault
* next step towards full mvc
* more encapsulation
* extracted boxworker as nodeworker into separate file
* splitter, segmentation and filters now use channel encoding
* fixed focus bug
* packages can now export templates
* streamlined box-connector-interface
* fixed roscore start dependency
* added clock to publish timestamps
* dynamic transform added (tf transform listener)
* added point cloud support
* added export to file module
* fixed deadlock
* Merge branch 'windows_port' of gitlab.cs.uni-tuebingen.de:buck/csapex into devel
* context menu for selection of boxes
* improved synchronized inputs mode
* removed unnecessary command
* towards pluginlib independance
* mad extractor manager independant of plugin_manager
* first step towards windows build
* improved profiling
* initial commit
  removed unnecessary packages from rabot
  small tweak in profiling widget
* added simple box profiling
* templates can now be saved (experimental);
  added custom (read: experimental) descriptor;
  converted robust matcher to read descriptors and keypoints
* start up speed increased
* refactored command and graph handling; bugs fixed
* streamlined template instantiation
* fixed a lot of group related bugs
* improved template handling; templates can be loaded from files
* reverted to rev 5196
* preparation for evaluation
* structured cs::APEX into modules; some refactorings
* refactored template handling
* forgot to commit
* improved box grouping; serialization as temporary templates
* moved id management to graph (towards sub-graphing)
* create demo / test package for boolean messages, continuing work on grouping
* reintroduced context menu for boxes
* users can now add waypoints to connections; added background grid; improved panning and growing of the workspace
* fixed bugs with ros-core dependency lag; added reset functions; extracted border padding into separate node
* refactored feature extraction parameters (config from vision_utils)
* forwarding plugin descriptions to tooltips
* reduced footprint of minimized boxes
* improved box movement
* registration cleanup and streamlining
* implemented headless mode
* scaled splash image :)
* fancy splash + unity icon
* removed debug output; improved file importer behaviour (url support)
* stupid bug fixed; added (de)select all functions
* allow drag / drop of external files (e.g. from nautilus) as file importer
* rqt -> drag and drop support for ros topics
* lots of bugfixes; cleanups; refactorings; began package for feature extraction and evaluation
* implemented ros export
* ported background subtraction; improved ros importing; several bugfixes and improvements;
* refactored grouping; deprecated categories in favor of tags
* improved grouping, still not complete
* fixed incomplete stuff
* box grouping; bug fixes; cleanup
* shell for meta box; toolbox for box selection
* uncomitted changes: refactorings to MVC, bugfixes
* context menu: no more namespaces; sorting
* improved startup; static mask correctly loaded
* usability: show splash screen -> faster response time
* fixed bug in 'static mask filter'
* file importer only shows files that can be opened
* eliminated cross dependencies; message providers are now plugins as well; extracted utility libraries from csapex_vision
* changed default config to ~/.csapex/default.apex;
  improved undo / redo handling ("dirty" flag)
* restructuring mostly done
* began restructuring vision_evaluator
* Contributors: Adrian Zwiener, Felix Widmaier, Niels Rohwer, Richard Hanten, Robert Pech, Sebastian Buck
