package art.chibi.homingenchant;

import org.bukkit.FluidCollisionMode;
import org.bukkit.Location;
import org.bukkit.Material;
import org.bukkit.Particle;
import org.bukkit.util.RayTraceResult;
import org.bukkit.Registry;
import org.bukkit.World;
import org.bukkit.block.Block;
import org.bukkit.enchantments.Enchantment;
import org.bukkit.entity.Arrow;
import org.bukkit.entity.Entity;
import org.bukkit.entity.LivingEntity;
import org.bukkit.event.EventHandler;
import org.bukkit.event.Listener;
import org.bukkit.event.entity.EntityShootBowEvent;
import org.bukkit.projectiles.ProjectileSource;
import org.bukkit.scheduler.BukkitRunnable;
import org.bukkit.util.Vector;

import io.papermc.paper.registry.RegistryAccess;
import io.papermc.paper.registry.RegistryKey;
import net.kyori.adventure.key.Key;

import java.util.*;

/**
 * Demonstrates a "military-grade" homing arrow using a simplified 3D A*
 * pathfinder,
 * plus speed boosts if the arrow has a likely hit (direct line-of-sight, close
 * range, etc.).
 * 
 * Will also re-check for a new target whenever the current target is no longer
 * valid.
 */
public class HomingEnchantmentListener implements Listener {

    // A* Search radius in blocks
    private static final int SEARCH_RADIUS = 60;

    // Distance threshold where we consider it "likely" the arrow can hit soon
    private static final double CLOSE_RANGE = 12.0;

    // Factor by which we boost speed in "likely hit" scenarios
    private static final double SPEED_BOOST_FACTOR = 8.5;

    @EventHandler
    public void onBowShoot(EntityShootBowEvent event) {
        if (!(event.getProjectile() instanceof Arrow)) {
            return; // Only handle standard arrows
        }
        if (event.getBow() == null) {
            return;
        }

        RegistryAccess registry = RegistryAccess.registryAccess();
        Registry<Enchantment> enchantments = registry.getRegistry(RegistryKey.ENCHANTMENT);

        // Check if the bow has the custom "chibi:aimbot" enchant
        if (!event.getBow().containsEnchantment(enchantments.get(Key.key("chibi:aimbot")))) {
            return;
        }

        // It's a homing arrow. Start the homing task.
        Arrow arrow = (Arrow) event.getProjectile();
        startMilitaryGradeHomingTask(arrow);
    }

    /**
     * Repeatedly updates the arrow's pathfinding and steers it around obstacles,
     * boosting speed when direct hits look likely, and re-finding a target if the
     * old one is invalid.
     */
    private void startMilitaryGradeHomingTask(final Arrow arrow) {
        new BukkitRunnable() {
            private LinkedList<Location> path = null; // A* path (list of block centers)
            private LivingEntity currentTarget = null;

            @Override
            public void run() {
                // Cancel if arrow is invalid
                if (!isArrowInFlight(arrow)) {
                    cancel();
                    return;
                }

                // Spawn some trail particles each tick
                spawnParticleTrail(arrow);

                // (1) If we have no target or the target is gone, find a new one
                if (currentTarget == null || !isValidTarget(currentTarget)) {
                    currentTarget = findNearestEntityInCone(arrow, 30.0, 90.0);
                    path = null; // reset path if new target
                }

                // (2) If still no target, let arrow fly normally for now
                if (currentTarget == null) {
                    return;
                }

                // (3) If we don't have a path yet, compute one
                if (path == null) {
                    path = computeAStarPath(arrow, currentTarget);
                }

                // (4) Either follow the path or try a direct shot
                if (path != null && !path.isEmpty()) {
                    followPath(arrow, currentTarget, path);
                } else {
                    // Path is null/empty => attempt direct shot
                    attemptDirectShot(arrow, currentTarget);
                }
            }
        }.runTaskTimer(HomingEnchantmentPlugin.getPlugin(HomingEnchantmentPlugin.class), 0L, 2L);
    }

    // --------------------------------------------------------------------------------------------
    // Particles, Checking Arrow Validity, & Basic Utilities
    // --------------------------------------------------------------------------------------------

    private void spawnParticleTrail(Arrow arrow) {
        // "glow-like" sparkle
        arrow.getWorld().spawnParticle(
                Particle.END_ROD,
                arrow.getLocation(),
                5,
                0.1, 0.1, 0.1,
                0.0);

        // fiery flame
        arrow.getWorld().spawnParticle(
                Particle.FLAME,
                arrow.getLocation(),
                8,
                0.1, 0.1, 0.1,
                0.0);
    }

    private boolean isArrowInFlight(Arrow arrow) {
        // Stop pathfinding if arrow is gone or stuck
        return arrow != null
                && !arrow.isDead()
                && arrow.isValid()
                && !arrow.isInBlock()
                && !arrow.isOnGround();
    }

    private boolean isValidTarget(LivingEntity entity) {
        return entity != null && !entity.isDead() && entity.isValid();
    }

    // --------------------------------------------------------------------------------------------
    // Attempting Direct Shots & Speed Boost
    // --------------------------------------------------------------------------------------------

    /**
     * If there's direct line-of-sight, steer arrow directly. Also try speed boost.
     */
    private void attemptDirectShot(Arrow arrow, LivingEntity target) {
        World world = arrow.getWorld();
        Location arrowLoc = arrow.getLocation();
        Location targetMid = getTargetMidpoint(target);

        if (hasLineOfSight(world, arrowLoc, targetMid)) {
            // Steer arrow directly
            steer(arrow, arrowLoc, targetMid);

            // Possibly boost speed if target is close enough
            increaseSpeedIfLikelyHit(arrow, arrowLoc, targetMid);
        }
        // else no direct line-of-sight => do nothing special this tick
    }

    /**
     * If the arrow is very close (CLOSE_RANGE) or line-of-sight is clear,
     * we multiply arrow velocity by SPEED_BOOST_FACTOR.
     */
    private void increaseSpeedIfLikelyHit(Arrow arrow, Location arrowLoc, Location targetLoc) {
        double dist = arrowLoc.distance(targetLoc);
        if (dist <= CLOSE_RANGE) {
            // Speed up
            Vector newVel = arrow.getVelocity().multiply(SPEED_BOOST_FACTOR);
            arrow.setVelocity(newVel);
        }
    }

    // Basic steering
    private void steer(Arrow arrow, Location from, Location to) {
        Vector dir = to.toVector().subtract(from.toVector()).normalize();
        double speed = arrow.getVelocity().length();
        arrow.setVelocity(dir.multiply(speed));
    }

    // --------------------------------------------------------------------------------------------
    // Following the A* Path
    // --------------------------------------------------------------------------------------------

    /**
     * If we have a path (list of nodes), move to the next node; if we can see
     * the final target, attempt direct shot & speed boost.
     */
    private void followPath(Arrow arrow, LivingEntity target, LinkedList<Location> path) {
        Location arrowLoc = arrow.getLocation();

        // Are we close to the next node in path?
        Location nextNode = path.getFirst();
        double distance = arrowLoc.distance(nextNode);

        if (distance < 1.0) {
            // Pop this node
            path.removeFirst();

            // If we exhausted the path, attempt direct shot
            if (path.isEmpty()) {
                attemptDirectShot(arrow, target);
                return;
            }
            // Otherwise, new nextNode
            nextNode = path.getFirst();
        }

        // Check if we can see the final target. If yes, skip path.
        Location finalNode = getTargetMidpoint(target);
        if (hasLineOfSight(arrow.getWorld(), arrowLoc, finalNode)) {
            attemptDirectShot(arrow, target);
            return;
        }

        // Otherwise, steer toward next path node
        steer(arrow, arrowLoc, nextNode);
    }

    // --------------------------------------------------------------------------------------------
    // Finding a Target
    // --------------------------------------------------------------------------------------------

    /**
     * Finds the nearest living entity in front of the arrow, within the given
     * angle cone (e.g. 90° total).
     */
    private LivingEntity findNearestEntityInCone(Arrow arrow, double radius, double totalAngleDegrees) {
        ProjectileSource source = arrow.getShooter();
        Vector arrowDir = arrow.getVelocity().normalize();

        double halfAngle = totalAngleDegrees / 2.0;
        double dotThreshold = Math.cos(Math.toRadians(halfAngle));

        LivingEntity nearest = null;
        double nearestDistSq = Double.MAX_VALUE;

        for (Entity entity : arrow.getNearbyEntities(radius, radius, radius)) {
            if (!(entity instanceof LivingEntity))
                continue;
            if (entity == source)
                continue; // skip the shooter

            LivingEntity candidate = (LivingEntity) entity;

            // Angle check
            Vector toCandidate = candidate.getLocation().toVector()
                    .subtract(arrow.getLocation().toVector())
                    .normalize();
            double dot = arrowDir.dot(toCandidate);
            if (dot < dotThreshold) {
                continue;
            }

            // Optional: check line-of-sight to the candidate
            if (!hasLineOfSight(arrow.getWorld(), arrow.getLocation(), getTargetMidpoint(candidate))) {
                continue;
            }

            // Among valid, pick nearest
            double distSq = arrow.getLocation().distanceSquared(candidate.getLocation());
            if (distSq < nearestDistSq) {
                nearestDistSq = distSq;
                nearest = candidate;
            }
        }
        return nearest;
    }

    private Location getTargetMidpoint(LivingEntity entity) {
        return entity.getLocation().add(0, entity.getHeight() * 0.5, 0);
    }

    // --------------------------------------------------------------------------------------------
    // A* Path Computation
    // --------------------------------------------------------------------------------------------

    private LinkedList<Location> computeAStarPath(Arrow arrow, LivingEntity target) {
        Location arrowLoc = arrow.getLocation();
        Block startBlock = arrowLoc.getBlock();

        Location targetLoc = target.getLocation();
        Block endBlock = targetLoc.getBlock();

        // Safety check: if start/end are super far, skip pathfinding
        if (startBlock.getLocation().distanceSquared(endBlock.getLocation()) > SEARCH_RADIUS * SEARCH_RADIUS * 9) {
            return null;
        }

        AStarPathfinder pathfinder = new AStarPathfinder(arrow.getWorld(), startBlock, endBlock, SEARCH_RADIUS);
        return pathfinder.findPath();
    }

    // --------------------------------------------------------------------------------------------
    // Ray-based "Line of Sight"
    // --------------------------------------------------------------------------------------------

    private boolean hasLineOfSight(World world, Location start, Location end) {
        if (start == null || end == null)
            return false;

        Vector dir = end.toVector().subtract(start.toVector());
        double dist = dir.length();
        dir.normalize();

        RayTraceResult result = world.rayTrace(
                start,
                dir,
                dist,
                FluidCollisionMode.NEVER,
                true,
                0.1,
                entity -> false // skip entities
        );

        // If null => no block was hit => line is clear
        if (result == null) {
            return true;
        }
        // If we hit a block or entity before "end" => not clear
        return false;
    }

    // --------------------------------------------------------------------------------------------
    // A* Classes
    // --------------------------------------------------------------------------------------------

    /**
     * Minimal container for block coords + cost for the A* search
     */
    private static class AStarNode {
        final int x, y, z;
        double gCost;
        double hCost;
        AStarNode parent;

        AStarNode(int x, int y, int z) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.gCost = Double.MAX_VALUE;
            this.hCost = 0;
        }

        double fCost() {
            return gCost + hCost;
        }
    }

    /**
     * Simple 3D A* block-based pathfinder that allows the arrow to navigate around
     * obstacles.
     */
    private static class AStarPathfinder {
        private final World world;
        private final Block startBlock;
        private final Block endBlock;
        private final int maxRadius;

        // Node cache for quick lookups
        private final Map<String, AStarNode> nodeCache = new HashMap<>();

        AStarPathfinder(World world, Block startBlock, Block endBlock, int maxRadius) {
            this.world = world;
            this.startBlock = startBlock;
            this.endBlock = endBlock;
            this.maxRadius = maxRadius;
        }

        LinkedList<Location> findPath() {
            // Convert to integer coords
            int sx = startBlock.getX(), sy = startBlock.getY(), sz = startBlock.getZ();
            int ex = endBlock.getX(), ey = endBlock.getY(), ez = endBlock.getZ();

            AStarNode startNode = getOrCreateNode(sx, sy, sz);
            startNode.gCost = 0;
            startNode.hCost = heuristic(startNode, ex, ey, ez);

            PriorityQueue<AStarNode> openSet = new PriorityQueue<>(Comparator.comparingDouble(AStarNode::fCost));
            openSet.add(startNode);

            while (!openSet.isEmpty()) {
                AStarNode current = openSet.poll();

                // If we've reached the end block
                if (current.x == ex && current.y == ey && current.z == ez) {
                    return buildPath(current);
                }

                // Explore neighbors
                for (AStarNode neighbor : getNeighbors(current, ex, ey, ez)) {
                    double newG = current.gCost + 1.0; // cost 1 per block step
                    if (newG < neighbor.gCost) {
                        neighbor.gCost = newG;
                        neighbor.hCost = heuristic(neighbor, ex, ey, ez);
                        neighbor.parent = current;

                        if (!openSet.contains(neighbor)) {
                            openSet.add(neighbor);
                        }
                    }
                }
            }
            // no path found
            return null;
        }

        private LinkedList<Location> buildPath(AStarNode endNode) {
            LinkedList<Location> path = new LinkedList<>();
            AStarNode current = endNode;
            while (current != null) {
                path.addFirst(new Location(
                        world,
                        current.x + 0.5,
                        current.y + 0.5,
                        current.z + 0.5));
                current = current.parent;
            }
            return path;
        }

        private double heuristic(AStarNode node, int ex, int ey, int ez) {
            double dx = (ex - node.x);
            double dy = (ey - node.y);
            double dz = (ez - node.z);
            // Euclidean distance
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        private List<AStarNode> getNeighbors(AStarNode node, int ex, int ey, int ez) {
            List<AStarNode> result = new ArrayList<>(6);
            int[][] offsets = {
                    { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }
            };
            for (int[] off : offsets) {
                int nx = node.x + off[0];
                int ny = node.y + off[1];
                int nz = node.z + off[2];

                if (!inRange(nx, ny, nz))
                    continue;
                if (!isPassable(nx, ny, nz))
                    continue;

                AStarNode neighbor = getOrCreateNode(nx, ny, nz);
                result.add(neighbor);
            }
            return result;
        }

        private boolean inRange(int x, int y, int z) {
            return Math.abs(x - startBlock.getX()) <= maxRadius
                    && Math.abs(y - startBlock.getY()) <= maxRadius
                    && Math.abs(z - startBlock.getZ()) <= maxRadius;
        }

        private boolean isPassable(int x, int y, int z) {
            Material mat = world.getBlockAt(x, y, z).getType();
            return !mat.isSolid();
        }

        private AStarNode getOrCreateNode(int x, int y, int z) {
            String key = x + "," + y + "," + z;
            return nodeCache.computeIfAbsent(key, k -> new AStarNode(x, y, z));
        }
    }
}
