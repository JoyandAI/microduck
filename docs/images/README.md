# Images

The `.svg` files here are **placeholders**. Each one renders as a dashed grey box naming the shot it
is standing in for, so the README lays out correctly before any photo exists.

To fill one in: drop a `.jpg`, `.png` or `.gif` next to it, point the README's `<img src>` at the new
file, and delete the placeholder. Keep the sizes — the layout is built around them.

| placeholder | what goes there | size |
|---|---|---|
| `hero.svg` | The robot, three-quarter view, in room light. The one image somebody sees first. | 1200×500 |
| `walk.svg` | A lap of the desk, side on. Short loop, no audio. | 560×340 |
| `roller.svg` | Wheels on, rolling past the camera. | 560×340 |
| `ground-pick.svg` | Beak to the floor, picking something up. | 560×340 |
| `standup.svg` | Face-down to standing, in one take. | 560×340 |

GitHub serves these through its own proxy, so a `.gif` over a couple of megabytes is slow on the
landing page — keep loops short and the palette small.

## Videos

A real video is better than a GIF for anything longer than a second or two, and GitHub hosts it for
you. Drop the file into the comment box of any issue or pull request (or the web editor's) — do not
commit it — and GitHub uploads it and hands back a URL like:

```
https://github.com/user-attachments/assets/8d7a530b-37f0-4d76-b47e-02cce6b326cb
```

**Where that URL goes depends on where in the page you are**, and this is the part that wastes an
afternoon:

- **In ordinary markdown**, the bare URL on a line of its own becomes a player. Nothing else needed.
- **Inside HTML** — which the "It does things" table is — markdown is not parsed at all, so a bare
  URL stays plain text and a link stays a link. Use the element instead:

  ```html
  <video src="https://github.com/user-attachments/assets/8d7a530b-…" controls muted loop width="100%"></video>
  ```

  **`src`, `controls`, `muted` and `width` are the only attributes that survive.** Tested against
  GitHub's own renderer (`POST https://api.github.com/markdown`, which runs the same sanitiser):
  `autoplay`, `loop`, `playsinline`, `preload` and `poster` are all stripped from the element.

### So a video cannot autoplay, and cannot loop

Not with `<video>`, and not with the player a bare URL produces either — both wait for a click.
**The only media that plays by itself in a README is an animated image**, which the browser
animates without asking: a GIF, or an animated WebP at a fraction of the size.

That is the route for the four tiles in "It does things", where the whole point is that they move
while somebody reads. Keep the clip to two or three seconds and treat it as a moving thumbnail:

```bash
# animated WebP — much smaller than a GIF at the same quality, and GitHub serves it fine
ffmpeg -i clip.mp4 -vf "fps=15,scale=560:-1:flags=lanczos" -loop 0 -q:v 55 walk.webp

# GIF, when you want the safest thing that exists
ffmpeg -i clip.mp4 -vf "fps=12,scale=560:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse" walk.gif
```

Then reference it as an ordinary image — `<img src="docs/images/walk.webp" width="100%">` — and it
plays, silently, forever, in a table cell.

Use `<video controls muted>` for the things a thumbnail cannot carry: a longer clip, or one with
sound. It is a click, and that is the trade.

Each cell in the README's table carries that line commented out beside its placeholder, so filling
one in is: upload, paste the id, delete the `<img>`.

Two consequences of GitHub hosting the file rather than the repo. It is **not in a clone** — the
README's videos are blank without a network, and a mirror of this repo still fetches them from
github.com. And GitHub caps attachment size, so a long clip belongs on the
[website](https://pollen-robotics.com/microduck) with a short one here.
