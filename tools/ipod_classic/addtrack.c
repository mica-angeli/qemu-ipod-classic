/*
 * Build an iPod music library on a mounted FAT32 image.
 *
 *   addtrack <mountpoint> <model> <track> [<track> ...]
 *
 * A <track> is either a bare path (the filename becomes the title, as the
 * first tests did) or "path|title|artist|album|track_nr|length_ms", which is
 * what mkmusic.sh passes after asking ffprobe.
 *
 * itdb_init_ipod() lays out iPod_Control and writes the device files; each
 * track is copied in and added to the master playlist; itdb_write() emits
 * iTunesDB and, for the models that need it, its hash.
 */

#include <gpod/itdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char *tag(const char *path, const char *what)
{
    /* Good enough for a test: the filename carries artist and title. */
    const char *base = strrchr(path, '/');
    return g_strdup(base ? base + 1 : path);
}

int main(int argc, char **argv)
{
    GError *err = NULL;
    Itdb_iTunesDB *db;
    Itdb_Playlist *mpl;
    int i;

    if (argc < 4) {
        fprintf(stderr, "usage: %s <mountpoint> <model> <mp3>...\n", argv[0]);
        return 2;
    }

    if (!itdb_init_ipod(argv[1], argv[2], "iPod", &err)) {
        fprintf(stderr, "init_ipod: %s\n", err ? err->message : "?");
        return 1;
    }
    printf("initialised %s as model %s\n", argv[1], argv[2]);

    db = itdb_parse(argv[1], &err);
    if (!db) {
        fprintf(stderr, "parse: %s\n", err ? err->message : "?");
        return 1;
    }

    mpl = itdb_playlist_mpl(db);
    if (!mpl) {
        mpl = itdb_playlist_new("iPod", FALSE);
        itdb_playlist_set_mpl(mpl);
        itdb_playlist_add(db, mpl, -1);
    }

    for (i = 3; i < argc; i++) {
        Itdb_Track *t = itdb_track_new();
        gchar **f = g_strsplit(argv[i], "|", 6);
        const char *path = f[0];

        if (g_strv_length(f) == 6) {
            t->title    = g_strdup(f[1]);
            t->artist   = g_strdup(f[2]);
            t->album    = g_strdup(f[3]);
            t->track_nr = atoi(f[4]);
            t->tracklen = atoi(f[5]);
        } else {
            t->title    = tag(path, "title");
            t->artist   = g_strdup("Linkin Park");
            t->album    = g_strdup("Collision Course");
            t->tracklen = 200000;        /* ms; only affects the progress bar */
        }
        t->filetype   = g_strdup("MPEG audio file");
        t->mediatype  = ITDB_MEDIATYPE_AUDIO;
        t->size       = 0;

        itdb_track_add(db, t, -1);
        itdb_playlist_add_track(mpl, t, -1);

        if (!itdb_cp_track_to_ipod(t, path, &err)) {
            fprintf(stderr, "copy %s: %s\n", path, err ? err->message : "?");
            return 1;
        }
        printf("added %s -> %s\n", path, t->ipod_path);
    }

    if (!itdb_write(db, &err)) {
        fprintf(stderr, "write: %s\n", err ? err->message : "?");
        return 1;
    }
    printf("wrote iTunesDB with %d track(s)\n", itdb_tracks_number(db));
    return 0;
}
