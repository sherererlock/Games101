//
// Created by goksu on 2/25/20.
//

#include <fstream>

#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
int Renderer::spp = 128;
int Renderer::currentRowIndex = 0;
std::mutex Renderer::RowIndexMutex;

void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            for (int k = 0; k < spp; k++) {

                float x_offset = get_random_float();
                float y_offset = get_random_float();
				float x = (2 * (i + x_offset) / (float)scene.width - 1) *
					imageAspectRatio * scale;
				float y = (1 - 2 * (j + y_offset) / (float)scene.height) * scale;

				Vector3f dir = normalize(Vector3f(-x, y, 1));
            
                framebuffer[m] += scene.castRay_(Ray(eye_pos, dir), 0) / spp;
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}

void Renderer::RenderMultithread(const Scene& scene)
{
	thread_count = std::thread::hardware_concurrency();
	thread_pools.reserve(thread_count);

	std::vector<Vector3f> framebuffer(scene.width * scene.height);

	float scale = tan(deg2rad(scene.fov * 0.5));
	float imageAspectRatio = scene.width / (float)scene.height;
	Vector3f eye_pos(278, 273, -800);

	// change the spp value to change sample ammount
	
	std::cout << "SPP: " << spp << " Thread Count: "<< thread_count << "\n";
	int row_count_per_thread = std::ceil((float)scene.height / (float)thread_count);
	int currentIndex = 0;
	for (int i = 0; i < thread_count; i++)
	{
		int bidx = currentIndex;
		int eidx = std::min(bidx + row_count_per_thread, scene.height);
		std::thread& t = thread_pools.emplace_back(Renderer::RenderRow, std::cref(scene), std::ref(framebuffer), std::cref(eye_pos), bidx, eidx, imageAspectRatio, scale);
		currentIndex = eidx;
	}

	for (int i = 0; i < thread_count; i++)
		thread_pools[i].join();

	//UpdateProgress(1.f);

	// save framebuffer to file
	FILE* fp = fopen("binary.ppm", "wb");
	(void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
	for (auto i = 0; i < scene.height * scene.width; ++i) {
		static unsigned char color[3];
		color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
		color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
		color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
		fwrite(color, 1, 3, fp);
	}
	fclose(fp);
}

void Renderer::RenderRow(const Scene& scene, std::vector<Vector3f>& framebuffer, const Vector3f& eye_pos, int beginRowIndex, int endRowRndex, float imageAspectRatio, float scale)
{
	int m = beginRowIndex * scene.width;
	for (int j = beginRowIndex; j < endRowRndex; j++)
	{
		for (uint32_t i = 0; i < scene.width; ++i) {
			// generate primary ray direction
			for (int k = 0; k < spp; k++) {

				float x_offset = get_random_float();
				float y_offset = get_random_float();
				float x = (2 * (i + x_offset) / (float)scene.width - 1) *
					imageAspectRatio * scale;
				float y = (1 - 2 * (j + y_offset) / (float)scene.height) * scale;

				Vector3f dir = normalize(Vector3f(-x, y, 1));

				framebuffer[m] += scene.castRay_(Ray(eye_pos, dir), 0) / spp;
			}
			m++;
		}

		RowIndexMutex.lock();
		currentRowIndex++;
		UpdateProgress(currentRowIndex / (float)scene.height);
		RowIndexMutex.unlock();
	}
}
