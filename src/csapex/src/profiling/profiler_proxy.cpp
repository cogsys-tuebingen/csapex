/// HEADER
#include <csapex/profiling/profiler_proxy.h>

/// COMPONENT
#include <csapex/io/channel.h>
#include <csapex/io/protcol/profiler_note.h>
#include <csapex/io/protcol/profiler_requests.h>

using namespace csapex;

ProfilerProxy::ProfilerProxy(io::ChannelPtr node_channel)
    : Profiler(true, 16),
      node_channel_(node_channel)
{
    observe(node_channel_->note_received, [this](const io::NoteConstPtr& note){
        if(const std::shared_ptr<ProfilerNote const>& cn = std::dynamic_pointer_cast<ProfilerNote const>(note)) {
            switch(cn->getNoteType()) {
            case ProfilerNoteType::EnabledChanged:
                enabled_changed(cn->getPayload<bool>(0));
                break;
            }
        }
    });
}

void ProfilerProxy::setEnabled(bool enabled)
{
    node_channel_->sendRequest<ProfilerRequests>(ProfilerRequests::ProfilerRequestType::SetEnabled, enabled);
}

void ProfilerProxy::updateInterval(std::shared_ptr<const Interval> &interval)
{
    Profile& prof = profiles_.at(interval->name());
    prof.addInterval(std::make_shared<Interval>(*interval));
}
