#include "math/Vec3.hpp"
#include "physics/World.hpp"

#include <SFML/Graphics.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr unsigned windowWidth = 1180;
constexpr unsigned windowHeight = 720;
constexpr float fixedDt = 1.0f / 60.0f;
constexpr float arenaMarginX = 44.0f;
constexpr float arenaTop = 74.0f;
constexpr float arenaBottomMargin = 34.0f;
constexpr std::size_t initialDynamicBodies = 46;
constexpr std::size_t maxBodies = 96;
constexpr std::size_t trailLength = 26;
constexpr float contactHighlightSeconds = 0.12f;
constexpr float nearContactTolerance = 2.0f;

struct RenderBody {
  World::RigidBodyId id;
  sf::Color color;
  bool isStatic = false;
  std::array<sf::Vector2f, trailLength> trail{};
  std::size_t trailHead = 0;
  std::size_t trailCount = 0;
  float contactFlash = 0.0f;
};

struct ContactMarker {
  sf::Vector2f a;
  sf::Vector2f b;
};

struct DemoScene {
  World world;
  std::vector<RenderBody> bodies;
  std::vector<ContactMarker> contacts;
  std::uint64_t steps = 0;
  float maxSpeed = 0.0f;
};

sf::FloatRect arenaBounds() {
  return sf::FloatRect(arenaMarginX, arenaTop,
                       static_cast<float>(windowWidth) - 2.0f * arenaMarginX,
                       static_cast<float>(windowHeight) - arenaTop - arenaBottomMargin);
}

sf::Color colorForBody(std::size_t index) {
  static const std::array<sf::Color, 8> palette = {
      sf::Color(49, 202, 156), sf::Color(242, 196, 73),  sf::Color(86, 154, 236),
      sf::Color(231, 93, 110), sf::Color(151, 119, 232), sf::Color(73, 190, 218),
      sf::Color(236, 137, 71), sf::Color(159, 213, 90),
  };

  return palette[index % palette.size()];
}

sf::Color withAlpha(sf::Color color, sf::Uint8 alpha) {
  color.a = alpha;
  return color;
}

void appendTrailPoint(RenderBody& renderBody, const RigidBody& body) {
  renderBody.trail[renderBody.trailHead] = sf::Vector2f(body.position.x, body.position.y);
  renderBody.trailHead = (renderBody.trailHead + 1) % trailLength;
  renderBody.trailCount = std::min(renderBody.trailCount + 1, trailLength);
}

RenderBody makeRenderBody(World::RigidBodyId id, sf::Color color, bool isStatic,
                          const RigidBody& body) {
  RenderBody renderBody;
  renderBody.id = id;
  renderBody.color = color;
  renderBody.isStatic = isStatic;
  appendTrailPoint(renderBody, body);
  return renderBody;
}

World::RigidBodyId addBody(DemoScene& scene, float mass, float radius, const Vec3& position,
                           const Vec3& velocity, const sf::Color& color, bool isStatic) {
  const World::RigidBodyId id = scene.world.createRigidBody(mass, radius);
  RigidBody& body = scene.world.rigidBody(id);
  body.position = position;
  body.velocity = velocity;
  body.material.restitution = isStatic ? 0.9f : 0.86f;
  body.material.linearDamping = isStatic ? 0.0f : 0.015f;

  scene.bodies.push_back(makeRenderBody(id, color, isStatic, body));
  return id;
}

void addStaticObstacles(DemoScene& scene) {
  const sf::FloatRect bounds = arenaBounds();
  const float centerX = bounds.left + bounds.width * 0.5f;
  const float centerY = bounds.top + bounds.height * 0.52f;

  addBody(scene, 0.0f, 34.0f, Vec3(centerX - 185.0f, centerY - 80.0f, 0.0f), Vec3(),
          sf::Color(104, 116, 128), true);
  addBody(scene, 0.0f, 26.0f, Vec3(centerX + 15.0f, centerY + 38.0f, 0.0f), Vec3(),
          sf::Color(112, 124, 136), true);
  addBody(scene, 0.0f, 38.0f, Vec3(centerX + 220.0f, centerY - 28.0f, 0.0f), Vec3(),
          sf::Color(98, 110, 122), true);
  addBody(scene, 0.0f, 22.0f, Vec3(centerX + 340.0f, centerY + 132.0f, 0.0f), Vec3(),
          sf::Color(118, 130, 142), true);
}

void addInitialDynamicBodies(DemoScene& scene) {
  const sf::FloatRect bounds = arenaBounds();
  constexpr std::size_t columns = 10;
  const float startX = bounds.left + 82.0f;
  const float startY = bounds.top + 72.0f;
  const float spacingX = 98.0f;
  const float spacingY = 72.0f;

  for (std::size_t i = 0; i < initialDynamicBodies; ++i) {
    const float radius = 9.0f + static_cast<float>((i * 5) % 12);
    const float mass = 0.7f + radius * 0.08f;
    const float row = static_cast<float>(i / columns);
    const float col = static_cast<float>(i % columns);
    const float offsetY = (i % 2 == 0) ? 0.0f : 18.0f;
    const float x = startX + col * spacingX;
    const float y = startY + row * spacingY + offsetY;

    const float xDirection = (i % 2 == 0) ? 1.0f : -1.0f;
    const float yDirection = ((i / 3) % 2 == 0) ? 1.0f : -1.0f;
    const float speedX = 78.0f + static_cast<float>((i * 13) % 74);
    const float speedY = 44.0f + static_cast<float>((i * 17) % 68);

    addBody(scene, mass, radius, Vec3(x, y, 0.0f),
            Vec3(xDirection * speedX, yDirection * speedY, 0.0f), colorForBody(i), false);
  }

  addBody(scene, 3.8f, 28.0f, Vec3(bounds.left + 72.0f, bounds.top + bounds.height * 0.5f, 0.0f),
          Vec3(315.0f, -18.0f, 0.0f), sf::Color(255, 230, 116), false);
}

DemoScene createScene() {
  DemoScene scene{World(Vec3(0.0f, 0.0f, 0.0f)), {}, {}, 0, 0.0f};
  scene.world.reserveRigidBodies(maxBodies);
  scene.bodies.reserve(maxBodies);
  scene.contacts.reserve(maxBodies * maxBodies / 2);

  addStaticObstacles(scene);
  addInitialDynamicBodies(scene);
  return scene;
}

void constrainToArena(World& world, std::vector<RenderBody>& bodies) {
  const sf::FloatRect bounds = arenaBounds();
  const float left = bounds.left;
  const float right = bounds.left + bounds.width;
  const float top = bounds.top;
  const float bottom = bounds.top + bounds.height;

  for (RenderBody& renderBody : bodies) {
    if (renderBody.isStatic) {
      continue;
    }

    RigidBody& body = world.rigidBody(renderBody.id);
    const float restitution = body.material.restitution;
    const float radius = body.shape().sphereRadius;

    if (body.position.x - radius < left) {
      body.position.x = left + radius;
      body.velocity.x = std::abs(body.velocity.x) * restitution;
      renderBody.contactFlash = contactHighlightSeconds;
    } else if (body.position.x + radius > right) {
      body.position.x = right - radius;
      body.velocity.x = -std::abs(body.velocity.x) * restitution;
      renderBody.contactFlash = contactHighlightSeconds;
    }

    if (body.position.y - radius < top) {
      body.position.y = top + radius;
      body.velocity.y = std::abs(body.velocity.y) * restitution;
      renderBody.contactFlash = contactHighlightSeconds;
    } else if (body.position.y + radius > bottom) {
      body.position.y = bottom - radius;
      body.velocity.y = -std::abs(body.velocity.y) * restitution;
      renderBody.contactFlash = contactHighlightSeconds;
    }
  }
}

void updatePresentationState(DemoScene& scene) {
  scene.contacts.clear();
  scene.maxSpeed = 0.0f;

  for (RenderBody& renderBody : scene.bodies) {
    const RigidBody& body = scene.world.rigidBody(renderBody.id);
    appendTrailPoint(renderBody, body);
    renderBody.contactFlash = std::max(0.0f, renderBody.contactFlash - fixedDt);
    scene.maxSpeed = std::max(scene.maxSpeed, body.velocity.length());
  }

  for (std::size_t i = 0; i < scene.bodies.size(); ++i) {
    RigidBody& a = scene.world.rigidBody(scene.bodies[i].id);
    for (std::size_t j = i + 1; j < scene.bodies.size(); ++j) {
      RigidBody& b = scene.world.rigidBody(scene.bodies[j].id);
      const Vec3 offset = a.position - b.position;
      const float nearDistance =
          a.shape().sphereRadius + b.shape().sphereRadius + nearContactTolerance;
      if (offset.lengthSq() <= nearDistance * nearDistance) {
        scene.bodies[i].contactFlash = contactHighlightSeconds;
        scene.bodies[j].contactFlash = contactHighlightSeconds;
        scene.contacts.push_back(
            {sf::Vector2f(a.position.x, a.position.y), sf::Vector2f(b.position.x, b.position.y)});
      }
    }
  }
}

void stepScene(DemoScene& scene) {
  scene.world.step(fixedDt);
  constrainToArena(scene.world, scene.bodies);
  updatePresentationState(scene);
  ++scene.steps;
}

bool spawnBodyAt(DemoScene& scene, const sf::Vector2f& position) {
  if (scene.bodies.size() >= maxBodies) {
    return false;
  }

  const sf::FloatRect bounds = arenaBounds();
  if (!bounds.contains(position)) {
    return false;
  }

  const std::size_t index = scene.bodies.size();
  const float radius = 10.0f + static_cast<float>((index * 7) % 11);
  const float mass = 0.8f + radius * 0.075f;
  const float angle = static_cast<float>((index * 47) % 360) * 3.14159265f / 180.0f;
  const float speed = 155.0f + static_cast<float>((index * 19) % 90);
  const Vec3 velocity(std::cos(angle) * speed, std::sin(angle) * speed, 0.0f);

  addBody(scene, mass, radius, Vec3(position.x, position.y, 0.0f), velocity,
          colorForBody(index + 3), false);
  return true;
}

bool loadDebugFont(sf::Font& font) {
  static constexpr std::array<const char*, 6> fontPaths = {
      "assets/Inter-Regular.ttf",
      "assets/Arial.ttf",
      "/System/Library/Fonts/Supplemental/Arial.ttf",
      "/System/Library/Fonts/Supplemental/Helvetica.ttf",
      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
      "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
  };

  for (const char* path : fontPaths) {
    if (std::filesystem::exists(path) && font.loadFromFile(path)) {
      return true;
    }
  }

  return false;
}

std::string titleString(bool paused, const DemoScene& scene, float timeScale, bool trailsEnabled) {
  std::ostringstream out;
  out << "Physics Engine Demo | " << (paused ? "paused" : "running") << " | bodies "
      << scene.bodies.size() << "/" << maxBodies << " | contacts " << scene.contacts.size()
      << " | scale " << timeScale << "x"
      << " | Space pause | N step | R reset | T trails | click spawn | Esc quit";
  if (!trailsEnabled) {
    out << " | trails off";
  }
  return out.str();
}

std::string hudString(const DemoScene& scene, float timeScale) {
  std::ostringstream out;
  out << "tick " << scene.steps << "   bodies " << scene.bodies.size() << " / " << maxBodies
      << "\ncontacts " << scene.contacts.size() << "   speed " << std::fixed << std::setprecision(2)
      << timeScale << "x";
  return out.str();
}

void drawGrid(sf::RenderWindow& window, const sf::FloatRect& bounds) {
  constexpr float spacing = 40.0f;
  sf::VertexArray lines(sf::Lines);
  const sf::Color major(42, 49, 58);
  const sf::Color minor(28, 33, 41);

  for (float x = bounds.left; x <= bounds.left + bounds.width + 0.5f; x += spacing) {
    const int index = static_cast<int>((x - bounds.left) / spacing);
    const sf::Color color = (index % 4 == 0) ? major : minor;
    lines.append(sf::Vertex(sf::Vector2f(x, bounds.top), color));
    lines.append(sf::Vertex(sf::Vector2f(x, bounds.top + bounds.height), color));
  }

  for (float y = bounds.top; y <= bounds.top + bounds.height + 0.5f; y += spacing) {
    const int index = static_cast<int>((y - bounds.top) / spacing);
    const sf::Color color = (index % 4 == 0) ? major : minor;
    lines.append(sf::Vertex(sf::Vector2f(bounds.left, y), color));
    lines.append(sf::Vertex(sf::Vector2f(bounds.left + bounds.width, y), color));
  }

  window.draw(lines);
}

void drawTrails(sf::RenderWindow& window, const DemoScene& scene) {
  for (const RenderBody& renderBody : scene.bodies) {
    if (renderBody.isStatic || renderBody.trailCount < 2) {
      continue;
    }

    sf::VertexArray strip(sf::LineStrip);
    for (std::size_t i = 0; i < renderBody.trailCount; ++i) {
      const std::size_t index =
          (renderBody.trailHead + trailLength - renderBody.trailCount + i) % trailLength;
      const float alphaFactor =
          static_cast<float>(i + 1) / static_cast<float>(renderBody.trailCount);
      strip.append(
          sf::Vertex(renderBody.trail[index],
                     withAlpha(renderBody.color, static_cast<sf::Uint8>(26.0f * alphaFactor))));
    }
    window.draw(strip);
  }
}

void drawContacts(sf::RenderWindow& window, const DemoScene& scene) {
  sf::VertexArray contactLines(sf::Lines);
  for (const ContactMarker& contact : scene.contacts) {
    contactLines.append(sf::Vertex(contact.a, sf::Color(255, 244, 180, 125)));
    contactLines.append(sf::Vertex(contact.b, sf::Color(255, 244, 180, 125)));
  }
  window.draw(contactLines);
}

void drawBodies(sf::RenderWindow& window, const DemoScene& scene) {
  sf::CircleShape glow;
  sf::CircleShape shape;
  glow.setPointCount(48);
  shape.setPointCount(48);

  for (const RenderBody& renderBody : scene.bodies) {
    const RigidBody& body = scene.world.rigidBody(renderBody.id);
    const float flash =
        contactHighlightSeconds > 0.0f
            ? std::clamp(renderBody.contactFlash / contactHighlightSeconds, 0.0f, 1.0f)
            : 0.0f;
    const float radius = body.shape().sphereRadius;

    glow.setRadius(radius + 6.0f + flash * 5.0f);
    glow.setOrigin(glow.getRadius(), glow.getRadius());
    glow.setPosition(body.position.x, body.position.y);
    glow.setFillColor(withAlpha(renderBody.color, renderBody.isStatic ? 24 : 34));
    window.draw(glow);

    shape.setRadius(radius);
    shape.setOrigin(radius, radius);
    shape.setPosition(body.position.x, body.position.y);
    shape.setFillColor(renderBody.isStatic ? sf::Color(88, 96, 106) : renderBody.color);
    shape.setOutlineThickness(renderBody.isStatic ? 3.0f : 2.0f);
    shape.setOutlineColor(flash > 0.0f ? sf::Color(255, 244, 188) : sf::Color(9, 12, 16));
    window.draw(shape);
  }
}

void drawHud(sf::RenderWindow& window, const sf::Font& font, const DemoScene& scene,
             float timeScale) {
  sf::Text title("Physics Engine Demo", font, 24);
  title.setFillColor(sf::Color(238, 242, 245));
  title.setPosition(28.0f, 17.0f);
  window.draw(title);

  sf::Text hud(hudString(scene, timeScale), font, 15);
  hud.setFillColor(sf::Color(202, 210, 218));
  hud.setLineSpacing(1.18f);
  hud.setPosition(static_cast<float>(windowWidth) - 230.0f, 12.0f);
  window.draw(hud);
}
} // namespace

int main(int argc, char** argv) {
  std::string captureDirectory;
  if (argc == 3 && std::string(argv[1]) == "--capture-dir")
    captureDirectory = argv[2];
  else if (argc != 1) {
    std::cerr << "usage: physics [--capture-dir PATH]\n";
    return 2;
  }
  if (!captureDirectory.empty())
    std::filesystem::create_directories(captureDirectory);
  unsigned captureFrame = 0;

  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Physics Engine Demo");
  window.setVerticalSyncEnabled(captureDirectory.empty());
  if (!captureDirectory.empty())
    window.setVisible(false);

  DemoScene scene = createScene();
  bool paused = false;
  bool trailsEnabled = true;
  constexpr std::array<float, 5> timeScales = {0.25f, 0.5f, 1.0f, 1.5f, 2.0f};
  std::size_t timeScaleIndex = 2;
  float accumulator = 0.0f;
  sf::Clock clock;

  sf::Font font;
  const bool hasDebugFont = loadDebugFont(font);

  sf::RectangleShape arena;
  const sf::FloatRect bounds = arenaBounds();
  arena.setPosition(bounds.left, bounds.top);
  arena.setSize(sf::Vector2f(bounds.width, bounds.height));
  arena.setFillColor(sf::Color(14, 17, 22));
  arena.setOutlineColor(sf::Color(88, 102, 116));
  arena.setOutlineThickness(2.0f);

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      } else if (event.type == sf::Event::MouseButtonPressed &&
                 event.mouseButton.button == sf::Mouse::Left) {
        const sf::Vector2f mousePosition =
            window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
        spawnBodyAt(scene, mousePosition);
      } else if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::Escape) {
          window.close();
        } else if (event.key.code == sf::Keyboard::Space) {
          paused = !paused;
        } else if (event.key.code == sf::Keyboard::R) {
          scene = createScene();
          accumulator = 0.0f;
          clock.restart();
        } else if (event.key.code == sf::Keyboard::T) {
          trailsEnabled = !trailsEnabled;
        } else if (event.key.code == sf::Keyboard::N && paused) {
          stepScene(scene);
        } else if (event.key.code == sf::Keyboard::Up || event.key.code == sf::Keyboard::Add ||
                   event.key.code == sf::Keyboard::Equal) {
          timeScaleIndex = std::min(timeScaleIndex + 1, timeScales.size() - 1);
        } else if (event.key.code == sf::Keyboard::Down ||
                   event.key.code == sf::Keyboard::Subtract ||
                   event.key.code == sf::Keyboard::Hyphen) {
          timeScaleIndex = timeScaleIndex == 0 ? 0 : timeScaleIndex - 1;
        }
      }
    }

    const float frameDt =
        captureDirectory.empty() ? std::min(clock.restart().asSeconds(), 0.25f) : 1.f / 30.f;
    const float timeScale = timeScales[timeScaleIndex];
    if (!paused) {
      accumulator += frameDt * timeScale;
    }

    while (accumulator >= fixedDt) {
      stepScene(scene);
      accumulator -= fixedDt;
    }

    window.setTitle(titleString(paused, scene, timeScale, trailsEnabled));

    window.clear(sf::Color(7, 9, 13));
    window.draw(arena);
    drawGrid(window, bounds);
    if (trailsEnabled) {
      drawTrails(window, scene);
    }
    drawContacts(window, scene);
    drawBodies(window, scene);
    if (hasDebugFont) {
      drawHud(window, font, scene, timeScale);
    }
    window.display();
    if (!captureDirectory.empty()) {
      sf::Texture texture;
      if (!texture.create(windowWidth, windowHeight))
        return 1;
      texture.update(window);
      std::ostringstream path;
      path << captureDirectory << "/frame-" << std::setw(3) << std::setfill('0') << captureFrame
           << ".png";
      if (!texture.copyToImage().saveToFile(path.str()))
        return 1;
      if (++captureFrame == 90)
        window.close();
    }
  }

  return 0;
}
