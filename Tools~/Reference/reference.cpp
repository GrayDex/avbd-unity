// Reference dump for the C# port: runs avbd-demo3d's solver headless on its own scenes and writes
// every body's pose (creation order) at the frames the tests compare. Built by build.sh.
#include <cstdio>
#include <vector>
#include "solver.h"
#include "scenes.h"

static bool recorded(int frame) { return frame <= 10 || frame % 10 == 0; }

int main(int argc, char **argv)
{
    const int frames = 600;
    FILE *out = fopen(argc > 1 ? argv[1] : "reference.bytes", "wb");
    int count = sceneCount - 1;
    fwrite(&count, 4, 1, out);
    Solver *solver = new Solver();
    for (int s = 1; s < sceneCount; s++)
    {
        solver->defaultParams();
        scenes[s](solver);
        std::vector<Rigid *> bodies;
        for (Rigid *b = solver->bodies; b; b = b->next) bodies.insert(bodies.begin(), b);
        int n = (int)bodies.size(), kept = 0;
        for (int f = 1; f <= frames; f++) kept += recorded(f);
        fwrite(&s, 4, 1, out);
        fwrite(&n, 4, 1, out);
        fwrite(&kept, 4, 1, out);
        for (int f = 1; f <= frames; f++)
        {
            solver->step();
            if (!recorded(f)) continue;
            fwrite(&f, 4, 1, out);
            for (Rigid *b : bodies)
            {
                fwrite(&b->positionLin, 4, 3, out);
                fwrite(&b->positionAng, 4, 4, out);
            }
        }
        printf("%-16s bodies %3d\n", sceneNames[s], n);
    }
    fclose(out);
    return 0;
}
