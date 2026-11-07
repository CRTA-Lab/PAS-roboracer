#version 130
#extension GL_ARB_shading_language_420pack : require

// Draws an 8-bit image using RGB values from a 256x1 palette texture.
//
// Patched copy of rviz_rendering's glsl120/indexed_8bit_image.frag.
// The upstream shader leaves both samplers on texture unit 0 until Ogre sets
// them after linking. Mesa >= 24 (Ubuntu 24.04) validates the program right
// after linking and reports "active samplers with a different type refer to
// the same texture image unit" (sampler2D vs sampler1D on unit 0).
// Explicit bindings give each sampler its own unit from the start, matching
// the material (eight_bit_image = 0, palette = 1).

varying vec2 UV;
layout(binding = 0) uniform sampler2D eight_bit_image;
layout(binding = 1) uniform sampler1D palette;
uniform float alpha;

void main()
{
  // The 0.999 multiply is needed because brightness value 255 comes
  // out of texture2D() as 1.0, which wraps around to 0.0 in the
  // palette texture.
  vec4 color = texture1D( palette, 0.999 * texture2D( eight_bit_image, UV ).x );
  gl_FragColor = vec4( color.rgb, color.a * alpha );
}
