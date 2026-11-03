// Do the FBX's blend-shape channels carry in-betweens? A channel has one keyframe per target shape
// (ufbx_blend_keyframe: shape + target_weight, the FBX "fullWeights"); a channel with more than one
// keyframe has in-between shapes. Godot's FBX importer keeps only the last keyframe of each channel
// (modules/fbx/fbx_document.cpp: keyframes[keyframes.count - 1]), so this reads the file with ufbx directly.
//
// build: clang -c -O1 <engine>/thirdparty/ufbx/ufbx.c -I<engine>/thirdparty/ufbx -o ufbx.o
//        clang++ -std=c++17 -O1 fbx_inbetweens.cpp ufbx.o -I<engine>/thirdparty/ufbx -o fbx_inbetweens
// usage: fbx_inbetweens <file.fbx> [mesh name]
#include <cstdio>
#include <cstring>
#include "ufbx.h"

int main(int argc, char **argv) {
	if (argc < 2) {
		std::fprintf(stderr, "usage: %s file.fbx [mesh name]\n", argv[0]);
		return 2;
	}
	ufbx_load_opts opts = {};
	ufbx_error err;
	ufbx_scene *scene = ufbx_load_file(argv[1], &opts, &err);
	if (!scene) {
		std::fprintf(stderr, "load failed: %s\n", err.description.data);
		return 1;
	}
	size_t channels = 0, multi = 0, shapes_total = 0, inbetween_shapes = 0;
	for (const ufbx_mesh *mesh : scene->meshes) {
		if (mesh->blend_deformers.count == 0)
			continue;
		if (argc > 2 && std::strcmp(mesh->name.data, argv[2]) != 0)
			continue;
		size_t mesh_channels = 0, mesh_multi = 0;
		for (const ufbx_blend_deformer *d : mesh->blend_deformers)
			for (const ufbx_blend_channel *c : d->channels) {
				++mesh_channels;
				shapes_total += c->keyframes.count;
				if (c->keyframes.count > 1) {
					++mesh_multi;
					inbetween_shapes += c->keyframes.count - 1;
					std::printf("  channel '%s' (mesh '%s'): %zu keyframes\n", c->name.data, mesh->name.data, c->keyframes.count);
					for (const ufbx_blend_keyframe &k : c->keyframes)
						std::printf("    target_weight=%g shape='%s' offsets=%zu\n", (double)k.target_weight, k.shape->name.data, k.shape->num_offsets);
				}
			}
		std::printf("mesh '%s': %zu vertices, %zu blend channels, %zu with in-betweens (more than one keyframe)\n", mesh->name.data,
				mesh->num_vertices, mesh_channels, mesh_multi);
		channels += mesh_channels;
		multi += mesh_multi;
	}
	std::printf("TOTAL: %zu channels, %zu keyframe shapes, %zu channels with in-betweens, %zu in-between shapes\n", channels,
			shapes_total, multi, inbetween_shapes);
	ufbx_free_scene(scene);
	return 0;
}
