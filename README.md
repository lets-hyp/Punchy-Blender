================================================================================

&#x20;                   PUNCHY RIG \& BLENDER ANIMATION SYSTEM

================================================================================



OVERVIEW

\--------------------------------------------------------------------------------

Bring the streamlined logic of Blockbench into Blender's professional animation 

environment. This tool retains the lightweight animation formatting of 

Blockbench while eliminating its clunky viewport navigation, restrictive curve 

editors, and timeline bottlenecks.





KEY FEATURES

\--------------------------------------------------------------------------------

\* Action Editor Centric Workflow:

&#x20; All animation data is built directly on Blender's native Action Editor. 

&#x20; Take full advantage of the Graph Editor, Dope Sheet, non-linear workflows, 

&#x20; and Bezier handles.



\* Fast Real-Time Preview:

&#x20; Test and iterate on animations instantly with Blender's smooth viewport 

&#x20; playback at target frame rates.



\* Marker-Driven Event, Sound \& Loop Control:

&#x20; Embed game events, audio, and playback settings straight into the timeline 

&#x20; using markers:

&#x20; - L:"<mode>"         -> Sets playback loop mode (e.g., L:"play\_once", L:"true", L:"hold\_on\_last\_frame").

&#x20; - S:"<sound\_id>"     -> Triggers sound effects at the exact frame/second (e.g., S:"sword\_in").

&#x20; - T:"<flag;>"        -> Outputs timeline event flags (e.g., dual\_handed;anim\_speed\_1;).



\* Direct Texture Item Referencing:

&#x20; Unlike Blockbench, you can preview held items with accurate 2.5D/3D voxel 

&#x20; extrusion directly from raw 2D item textures:

&#x20; - Select either of the two dedicated item helper objects attached to the 

&#x20;   rig (itemgrip\_right / itemgrip\_left).

&#x20; - Swap the texture input directly in the modifier/shader setup to preview 

&#x20;   any Minecraft item.

&#x20; - Adjust texture resolution, position, rotation, and scale offsets via the 

&#x20;   modifier stack to match in-game Minecraft holding positions 1:1 before 

&#x20;   animating.





QUICK START

\--------------------------------------------------------------------------------

1\. Select the Armature: Switch to Pose Mode with your Punchy rig selected\[cite: 5].

2\. Create/Select an Action: Open the Action Editor / Dope Sheet and animate.

3\. Add Markers: Press 'M' on the timeline to place event markers (L:, S:, T:).

4\. Export: Open the 'Punchy Exporter' tab in the 3D Viewport sidebar (N panel) 

&#x20;  and click 'Export JSON'\[cite: 5].

&#x20;  \* Enable "Export Action Timeline MetaData" in the file dialog to preserve 

&#x20;    editable Bezier curves for future re-imports\[cite: 5].





SUPPORT \& FEEDBACK

\--------------------------------------------------------------------------------

If you encounter any bugs, unexpected behavior, or have feature requests, 

please report them to the developer:



\* Punchy Wiki:

&#x20; https://github.com/punchy-guys/punchy-wiki/wiki\[cite: 5]



\* Punchy Blender Wiki \& Issues:

&#x20; https://github.com/Hamza-inc/Punchy-Blender/wiki\[cite: 5]

&#x20; 

&#x20; You can Remove That page

&#x20; Main Page is "Layout - Animator"

